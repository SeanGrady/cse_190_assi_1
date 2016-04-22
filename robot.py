#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTexture, moveService
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
from std_msgs.msg import Bool, String, Float32
from read_config import read_config
from math import exp


class Robot():
        def __init__(self):
                print("aa-self")
                self.config = read_config()

                rospy.init_node('Robot', anonymous=True)
                self.probMatrix = []
                self.row = [1.0/20, 1.0/20, 1.0/20, 1.0/20, 1.0/20]
                self.probMatrix.append(deepcopy(self.row))
                self.probMatrix.append(deepcopy(self.row))
                self.probMatrix.append(deepcopy(self.row))
                self.probMatrix.append(deepcopy(self.row))

            #to keep track of number of moves
                self.index = 0
                self.moveList = self.config["move_list"]

                #pos
                self.currPos = self.config["starting_pos"]#if I accessing the starting pos here
            #does not it implicitly mean I know my position?
                self.currPosX = self.currPos[0]
                self.currPosY = self.currPos[1]

            #prob of moves
                self.correctMoveProb = self.config["prob_move_correct"]
                self.incorrectMoveProb = (1- self.correctMoveProb)/4.0

            #bounds for the prob matrix
                self.matrixColumnBound = len(self.probMatrix[0]) -1 # 0 1 2 3 4 --> len == 5
                self.matrixRowBound = len(self.probMatrix) -1 # 0 1 2 3 --> len == 4

            #texture Map
                self.textureMap = self.config['texture_map']
                print self.config['texture_map']
                print str(self.textureMap[0][0])


            #temp map and std dev
                self.tempMatrix = self.config["pipe_map"]
                self.tempStdDev = self.config["temp_noise_std_dev"]

                rospy.Subscriber("/temp_sensor/data", temperatureMessage, self.callback)

                #rospy.sleep(5)

                self.pub = rospy.Publisher("/temp_sensor/activation", Bool, queue_size=10)

                rospy.wait_for_service('requestTexture')

                self.requestTextureProxy = rospy.ServiceProxy('requestTexture', requestTexture)

                rospy.wait_for_service("moveService")

                self.requestMoveProxy = rospy.ServiceProxy('moveService', moveService)

                self.publishTexture = rospy.Publisher('/results/texture_data', String, queue_size=10)

                self.publishTemp = rospy.Publisher('/results/temperature_data', Float32, queue_size=10)

                self.publishProb = rospy.Publisher('/results/probabilities', RobotProbabilities, queue_size=10)
                self.publishStop = rospy.Publisher('/map_node/sim_complete', Bool, queue_size=10)

                #self.txtReading = "S"

                self.temperatureSensorUpdate()

                rospy.spin()

        def temperatureSensorUpdate(self):
                print("Publish activation")
                rospy.sleep(5)
                self.pub.publish(Bool(True))
                #rospy.sleep(1)
            #so now callback function can be started calling by the system

        def callback(self, data):





                #print("temp reading: ")
                tempReading = data.temperature
                #print(tempReading)

                self.probMatrix = self.getJointPTempAndX(tempReading, 0, 0)
                #print "Prob Matrix"
                #print self.probMatrix
                self.probMatrix = self.textureSensorUpdate()


                if self.index == len(self.config["move_list"]):
                        #publish to the temperature topic
                        self.publishTemp.publish(data.temperature)
                        #publish to the texture topic
                        self.publishTexture.publish(self.txtReading.data)

                        temp = []
                        for i in range ( len(self.probMatrix) ):
                                for j in range( len( self.probMatrix[i] ) ):
                                        temp.append(self.probMatrix[i][j])



                        #publish the updated prob matrix
                        self.publishProb.publish(temp)

                        self.publishStop.publish(True) #we must quit calling the callback function!!

                        rospy.sleep(1)
                        #or do we kill the function
                        rospy.signal_shutdown(self)

                #normalizatnCst = self.getNormalizationConstant(tempReading, False)#data is the temperature reading

                #print self.probMatrix

                self.moveSensorUpdate()

                self.publishTemp.publish(data.temperature)



                        #publish to the texture topic
                self.publishTexture.publish(self.txtReading.data)

                #publish the updated prob matrix
                temp = []
                for i in range ( len(self.probMatrix) ):
                        for j in range( len( self.probMatrix[i] ) ):
                                temp.append(self.probMatrix[i][j])



                self.publishProb.publish(temp)



                #print self.probMatrix




                #self.moveSensorUpdate()




                #self.rate = rospy.Rate(10) # 10hz

        #while not rospy.is_shutdown():
                #hello_str = "hello world %s" % rospy.get_time()
                        #rospy.loginfo(hello_str)

                #publish to the temperature topic
                        #self.publishTemp.publish(self.tempReading)
                        #publish to the texture topic
                        #self.publishTexture.publish(self.txtReading)

                        #publish the updated prob matrix
                        #self.publishProb.publish(self.probMatrix)

                        #self.rate.sleep()

                #call back only up until you need to move once that is done you are done!
                #if index < numOfMoves:
                #       callback(data)


                #why do we caluclate the N over all boxes for temperature but not for texture?
            #is it because one is continuous and the other is discrete?
            #since my reading is can itself be incorrect so I must sum up over all
            #possibilities

        def getJointPTempAndX(self, tempReading, x0 ,y0):
                #print("sss")
                jointMatrix = []
                #print("tempReading")
                #print(tempReading)
                row = [0.0, 0.0, 0.0, 0.0, 0.0]
                jointMatrix.append(deepcopy(row))
                jointMatrix.append(deepcopy(row))
                jointMatrix.append(deepcopy(row))
                jointMatrix.append(deepcopy(row))

                normal = 0.0
                for i in range ( len(self.probMatrix) ):
                        for j in range( len( self.probMatrix[i] ) ):

                                if self.tempMatrix[i][j] == "H":
                                        gridTemp = 40
                                elif self.tempMatrix[i][j] == "C":
                                        gridTemp = 20
                                else:
                                        gridTemp = 25

                                error = gridTemp - tempReading
                            #if the error is high then the P of T given X
                            #will be really small
                                #print("self.tempStdDev")
                                #print(self.tempStdDev)

                                constant = 1.0/( m.sqrt( 2 * m.pi) * self.tempStdDev )

                                eExp = m.exp(-0.5*( (m.pow(error, 2) )/( m.pow(self.tempStdDev, 2) ) ) )
                                #print("eExp")
                                #print(eExp)

                                probTGivenX = constant * eExp

                                prior = self.probMatrix[i][j]

                                #print("prioir")
                                #print(prior)

                                jointMatrix[i][j] = probTGivenX * prior
                                #print("jointMatrix[i][j]")
                                #print(jointMatrix[i][j])

                                #normal = normal + jointMatrix[i][j]
                #print("normal")
                #print(normal)
                                #print(jointMatrix)
                for x in range( len( self.probMatrix ) ):
                        for y in range( len( self.probMatrix[x] ) ):
                                normal += jointMatrix[x][y]

                #print("normal")
                #print(normal)

                #for x
                #for y
                for x in range( len( self.probMatrix ) ):
                        for y in range( len( self.probMatrix[x] ) ):
                                #print("jointMatrix[x][y]/normal")
                                #print(jointMatrix[x][y]/normal)

                                jointMatrix[x][y] = jointMatrix[x][y]/normal

                #print jointMatrix
                return jointMatrix


        def textureSensorUpdate(self):

                #get the texture reading
                textureResponse = self.requestTextureProxy()
                self.txtReading = textureResponse
                normal = 0.0

                jointMatrix1 = []
                #print("txtReading")
                #print(txtReading)
                row = [0.0, 0.0, 0.0, 0.0, 0.0]
                jointMatrix1.append(deepcopy(row))
                jointMatrix1.append(deepcopy(row))
                jointMatrix1.append(deepcopy(row))
                jointMatrix1.append(deepcopy(row))

            #calculate the N for this reading
            #normalizatnCnst = self.getNormalizationConstant(txtReading, True)

            #walk through each grid in the prob map
                #print "Prob Matrix"
                #print self.probMatrix
                #print "self.textureMap[i][j] == txtReading"
                #print self.textureMap
                #print txtReading
                for i in range(len(self.textureMap)):

                        for j in range( len( self.textureMap[i] ) ):
                                #print str(self.textureMap[i][j])
                                #print str(txtReading)
                                if str(self.textureMap[i][j]) == str(self.txtReading.data):
                                        jointMatrix1[i][j] = self.config["prob_tex_correct"] * self.probMatrix[i][j]
                                        #normal = normal + jointMatrix[i][j]
                                        #print self.config["prob_tex_correct"]
                                else:
                                        #print self.correctMoveProb
                                        #print 1 - self.config["prob_tex_correct"]
                                        jointMatrix1[i][j] = ( 1 - self.config["prob_tex_correct"] ) * self.probMatrix[i][j]
                                        #normal = normal + jointMatrix[i][j]
                #print "Joint Matrix 1 "
                #print jointMatrix1

                for i in range(len(jointMatrix1)):
                        for j in range( len(jointMatrix1[i] ) ):
                                normal = normal + jointMatrix1[i][j]


                for i in range(len(jointMatrix1)):
                        for j in range( len( jointMatrix1[i] ) ):
                                jointMatrix1[i][j] = jointMatrix1[i][j]/normal

                #print "Joint Matrix 2"
                #print jointMatrix1
                return jointMatrix1

        def moveSensorUpdate(self):
                #moveSensorClient()
                #this is the value robot should move by, this is an array with 2 values

                jointMatrix1 = []
                #print("txtReading")
                #print(txtReading)
                row = [0.0, 0.0, 0.0, 0.0, 0.0]
                jointMatrix1.append(deepcopy(row))
                jointMatrix1.append(deepcopy(row))
                jointMatrix1.append(deepcopy(row))
                jointMatrix1.append(deepcopy(row))


                moveValue = self.config['move_list'][self.index]
                self.requestMoveProxy(moveValue)
                #self.probUpdateGivenMove(moveValue)
                col = len (self.probMatrix[0])
                row = len (self.probMatrix)

                for i in range( len (self.probMatrix[0]) ):
                        for j in range( len(self.probMatrix)):
                                for z in  self.config['possible_moves'] :
                                        if z == moveValue:
                                                jointMatrix1[j][i] += self.config['prob_move_correct'] * self.probMatrix[(j - z[0]) % row][ (i - z[1]) % col]
                                        else:
                                                jointMatrix1[j][i] += (1 - self.config['prob_move_correct'])/4 * self.probMatrix[ (j - z[0]) % row ] [ (i - z[1]) % col ]

                #print jointMatrix1
                #jointMatrix1

                #print "probMatrix"
                #print probMatrix


                self.index += 1
                self.probMatrix =  jointMatrix1
                #print self.probMatrix
                #increment the index so that next time this funciton is
                #called we go to access the next move
                #we must send some value to the move service to tell it that what
                #is the move that we want

                #when I make a move request the robot
                #moves to the position with certain probabilites
                #now I need to use this value to update my probability
                #map I dont need to worry about moving the robot
                #the map_sensor program is taking care of that




if __name__ == '__main__':
        print("aa")
        robot = Robot()

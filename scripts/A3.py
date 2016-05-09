#!/usr/bin/env python
from math import *

class likelihood_weighting:
	def __init__(self, *args):
		self.pB = 0.5
		self.n = 10
		self.err = 0.25
		self.given_b_val = 1
		self.given_b_idx = 8
		self.z = 128
		self.sum = 0
		# self.end_it = [1 for x in range(self.n)]
		b_vector = list(itertools.product([0, 1], repeat=n))
		sum_factor = 0

		for b in b_vector:
			if(b[self.given_b_idx] != given_b_val):
				continue
			fB = sum(pow(2,k)*b[k] for k in range(self.n))
			sum_factor += pow(self.err, 128-fB)


		fB = sum(pow(2,i), b[i])
		pB_given_Z = sum(Z_given_allBs*)
		pZ_given_allBs

	# def _get_sum_factor(idx, bit_val):
	# 	global b, sum_factor, poll_idx

	# 	b[idx] = bit_val

	# 	if idx == 0:
	# 		fB = sum(pow(2,k)*b[k] for k in range(self.n))
	# 		sum_factor += pow(err, 128-fB)
	# 		if fB == self.end_it:
	# 			return sum_factor

	# 	if idx == poll_idx:
	# 		if b[poll_idx] == 0:
	# 			b = [0 for x in range(poll_idx-1, self.n)]
	# 			_get_sum_factor(poll_idx, 1)
	# 		else:
	# 			poll_idx += 1
	# 			_get_sum_factor(poll_idx, 0)
	# 	else:
	# 		_get_sum_factor(idx-1, 0)





if __name__ == '__main__':
	lw = likelihood_weighting()
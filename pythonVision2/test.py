import cgkit.all as cg
# import numpy as np
from numpy import *
import userdefined as us
# q0 = cg.quat([1,0,0,0])
# # print q 
# q1 = cg.quat([0,1,0,0])

# # def quat2list(q):
# # 	return 

# for i in range(0,101):
# 	# qn1 = cg.slerp(0.01*i+0.01, q0, q1, shortest=True)
# 	qn = cg.slerp(0.01*i, q0, q1, shortest=True)
	

# 	print i,qn.w
# dic = {(1,2,3):(1,2,3),
# 		(2,4,1):(4,12,2),(5,2,4):(2,3,4)}

# for i in dic:
# 	print i

# print dic[0]
	

R = array(range(0,9)).reshape([3,3])
print R
print us.vee(R)
print R.T
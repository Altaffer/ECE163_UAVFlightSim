#!/usr/bin/env python
# coding: utf-8

# In[12]:


m = [[0,0,0],[0,0,0],[0,0,0]]
I = [[1,0,0],[0,1,0],[0,0,1]]
pqr = [[1],[1],[1]]
del_t=0.001
w_x = [[0, 1, -1],[-1, 0, 1],[1, -1, 0]]
a = [[1,0,0],[0,1,0],[0,0,1]]
b = [[1,0,0],[0,1,0],[0,0,1]]
f = [[1, 0.001, -0.001],[-0.001, 1, 0.001],[0.001, -0.001, 1]]


for i in range(len(100)):
    m = mm.add(m, mm.scalarMultiply(del_t, mm.multiply([[1, math.sin(m[2][0])*math.tan(m[1][0]), math.cos([2][0])*math.tan(m[1][0])], [0, math.cos(m[2][0]), -math.sin(m[2][0])], [0, (math.sin(m[2][0])/math.cos(m[1][0])), (math.cos(m[2][0])/math.cos(m[1][0]))]], pqr)))    
    a = mm.add(a, mm.scalarMultiply(del_t, mm.multiply(w_x, a)))
    a = mm.add(a, mm.scalarMultiply(.5, mm.multiply(mm.subtract(I,mm.multiply(a,mm.transpose(a))), a)))
    b = mm.multiply(f, b)

print(m)
print("dcm2Euler of a: ", dcm2Euler(a), " dcm2Euler of b: "dcm2Euler(b))


# In[ ]:





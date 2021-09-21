from tf import transformations

a=1.5707
b=transformations.quaternion_from_euler(0,0,a,'sxyz')
print(b[0])
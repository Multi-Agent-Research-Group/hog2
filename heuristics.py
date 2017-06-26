from math import sqrt

SQRT_13=sqrt(13)
SQRT_10=sqrt(10)
SQRT_5=sqrt(5)
SQRT_2=sqrt(2)

def h4(p):
  dx=abs(p[0][0]-p[1][0])
  dy=abs(p[0][1]-p[1][1])
  return _h4(dx,dy)

def _h4(dx,dy,result=0):
  return result+dx+dy

def h8(p):
  dx=abs(p[0][0]-p[1][0])
  dy=abs(p[0][1]-p[1][1])
  return _h8(dx,dy)

def _h8(dx,dy,result=0):
  if dx>dy:
    dy,dx=dx,dy # swap
  if dy>=dx:
    d=dx #diagonal amt
    result += dx*SQRT_2
    dy -= dx
    dx -= dx
  return _h4(dy,dx,result)

def h24(p):
  dx=abs(p[0][0]-p[1][0])
  dy=abs(p[0][1]-p[1][1])
  return _h24(dx,dy)

def _h24(dx,dy,result=0):
  if dx>dy:
    dy,dx=dx,dy # swap
  diff=dy-dx
  if diff >=1:
    s5=min(diff,dx)
    result += s5*SQRT_5
    dy -= s5*2 #up/down 2
    dx -= s5 #over 1
  return _h8(dx,dy,result)

if __name__ == "__main__":
  a=[[(0,0),(1,1),2,sqrt(2),sqrt(2)],
    [(0,0),(1,2),3,sqrt(2)+1,sqrt(5)],
    [(0,0),(2,4),6,sqrt(2)*2+2,sqrt(5)*2],
    [(0,0),(3,5),8,sqrt(2)*3+2,sqrt(5)*2+sqrt(2)],
    [(0,0),(2,5),7,sqrt(2)*2+3,sqrt(5)*2+1]]
  for i in a:
    print i[:2]
    print i[2],h4(i)
    print i[3],h8(i)
    print i[4],h24(i)
    

import pygame
from math import pi


pygame.init()

f = open("maptest.txt", 'r')

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

size = [300, 400]
screen = pygame.display.set_mode(size)

screen.fill(WHITE)
f1 = f.readlines()
x = 0;
for line in f1:
	line.split()
	y = 0
	for char in line:
		if char == "X":
			pygame.draw.circle(screen, BLACK, [y, x], 3)
		if char == "O":
			pygame.draw.circle(screen, RED, [y, x], 3)
		y = y + 1
	x = x + 1
pygame.display.flip()
while 1==1:
	print("hi")
	

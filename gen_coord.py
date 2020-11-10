import random

matricula = 201813232

random.seed(matricula)
x = 0
y = 0

while x**2 + y**2 > 4**2 or x**2 + y**2 < 2**2:
    x = random.random() * 8 - 4
    y = random.random() * 8 - 4

print(x, y)
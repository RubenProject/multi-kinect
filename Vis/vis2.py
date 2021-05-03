import matplotlib.pyplot as plt

try:
    fp = open('chess.txt');
except:
    print("")

try:
    fp = open('Vis/chess.txt');
except:
    print("failure to open files")

x = []
y = []
z = []

for line in fp.readlines():
    x.append(float(line.split(' ')[0]))
    y.append(float(line.split(' ')[1]))
    z.append(float(line.split(' ')[2]))

plt.plot(y, z, 'ro')

plt.xlabel('y')
plt.ylabel('z')
plt.show()


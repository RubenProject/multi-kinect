import matplotlib.pyplot as plt

try:
    fp0 = open('plot0.txt');
    fp1 = open('plot1.txt');
except:
    print("")

try:
    fp0 = open('Vis/plot0.txt');
    fp1 = open('Vis/plot1.txt');
except:
    print("failure to open files")

x = []
z = []

for line in fp0.readlines():
    x.append(float(line.split(' ')[0]))
    z.append(float(line.split(' ')[1]))

plt.plot(x, z, 'ro')

x = []
z = []

for line in fp1.readlines():
    x.append(float(line.split(' ')[0]))
    z.append(float(line.split(' ')[1]))

plt.plot(x, z, 'bo')


plt.xlabel('x')
plt.ylabel('z')
plt.show()


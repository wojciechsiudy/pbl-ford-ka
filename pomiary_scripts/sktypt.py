import matplotlib.pyplot as plt
import os, re, math

def clearData(line):
    line = re.sub('.*Range: ', '', line)
    line = re.sub(' m.*', '', line)
    return line

dalmierze = []
mbry = []
rmsy = []

files = os.listdir(os.getcwd())

for filename in files: #dla każdego pomiaru dalmirzowego
   if "txt" in filename:
    sigma = 0
    sigmaKwadrat = 0
    wartOczekiwana = float(re.sub('.txt', '', filename))
    dalmierze.append(wartOczekiwana)
    with open(os.path.join(os.getcwd(), filename), 'r') as f:
        n = 0
        print(f)
        for line in f.readlines(): #pomiary z uwb
            print(line)
            if line != '':
                n += 1
                pomiar = float(clearData(line).strip())
                delta=pomiar-wartOczekiwana
                deltaKwadrat = delta * delta
        sigma += delta
        sigmaKwadrat += deltaKwadrat
    mbr = sigma / n
    rmse = math.sqrt(sigmaKwadrat / n)
    dalmierze.sort()
    mbry.insert(dalmierze.index(wartOczekiwana), mbr)
    rmsy.insert(dalmierze.index(wartOczekiwana), rmse)

plt.ylabel('min bias error')
plt.ylabel('mbr')
plt.xlabel('odległość')    
plt.plot(dalmierze, mbry)
plt.show()

#with open(os.path.join(os.getcwd(), "3.978.txt"), 'r') as f:
#   wyniki = []
#   for line in f.readlines():
#      wyniki.append(float(clearData(line).strip()))

#plt.plot(wyniki)
#plt.show()

import numpy as np 
import csv

hand = np.array([3,0,0])
elbow = np.array([4,0,0])
shoulder = np.array([4,0,1])

oberarmvec  = shoulder-elbow
unterarmvec = elbow-hand

elbowrad = np.arccos(np.dot(oberarmvec,unterarmvec)/ (np.sqrt((oberarmvec*oberarmvec).sum())*np.sqrt((unterarmvec*unterarmvec).sum())))
elbowangle = elbowrad * 360 / 2 / np.pi
user = "test"
zeit = 1234312

with open('armlaengen.csv','a', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([f'{user} {zeit} elbogenwinkel: {elbowangle}'])

print(oberarmvec)
print(unterarmvec)
print(f'Winkel {elbowangle}')
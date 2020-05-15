import numpy as np
import csv
from TimedData import TimedData

def csvLoadTransform(filename, timeCol, posCol, attCol, td, pos, att, velCol = None, rorCol = None, vel = None, ror = None, timescale = 1, delimiter = ',', start = 0):
    posID = td.getColIDs(pos)
    attID = td.getColIDs(att)
    if vel is not None:
        velID = td.getColIDs(vel)
    if ror is not None:
        rorID = td.getColIDs(ror)
    # print(attID)
    if( td.last == (-1) ):
        with open(filename, 'rb') as csvfile:
            spamreader = csv.reader(csvfile, delimiter = delimiter, quotechar='|')
            counter = 0
            for row in spamreader:
                # print row
                try:
                    if counter < start:
                        counter += 1
                        continue
                    float(row[timeCol])
                    td.append();
                    td.d[td.last, td.timeID] = row[timeCol]*timescale;
                    td.d[td.last, posID] = [row[i] for i in posCol]
                    td.d[td.last, attID] = [row[i] for i in attCol]
                    if velCol is not None:
                        td.d[td.last, velID] = [row[i] for i in velCol]
                    if rorCol is not None:
                        td.d[td.last, rorID] = [row[i] for i in rorCol]
                    counter = counter + 1
                except ValueError:
                    print "Ignoring line " + str(counter)
        print("loading " + filename + " as transform, found " + str(counter) + " entries")
    else:
        print('Implement functionality when timedata is not empty');

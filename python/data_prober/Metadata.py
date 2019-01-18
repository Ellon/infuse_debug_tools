import matplotlib.pyplot as plt
import numpy as np
from MetadataFormat import MetadataFormat

class Metadata:

    metadataFormat = MetadataFormat()

    def __init__(self):
        pass

    def __repr__(self):
        return "Metadata()"

    def __str__(self):
        return self.metadataFormat.__str__()

    def parse_metadata(self, metadataFormatFilename, metadataFilename):

        self.metadataFormat.parse_metadata_format_file(metadataFormatFilename)

        setattr(self, "index", [])
        for name in self.metadataFormat.dataFields:
            setattr(self, name, [])
        
        metadataFile = open(metadataFilename, "r")

        nbAttributes = len(self.__dict__.keys())
        indexCount = 0
        for line in metadataFile:
            self.index.append(indexCount)
            words = line.split()
            if len(words) != nbAttributes - 1:
                raise Exception("Error : Number of columns isn't equal to the number of metadata fields descripted in " + metadataFilename)
            count = 0
            for word in words:
                getattr(self, self.metadataFormat.dataFields[count]).append(float(word))
                count += 1
            indexCount += 1

    def get(self, field):
        if type(field) is int:
            return getattr(self, self.metadataFormat.dataFields[field])
        elif type(field) is str:
            return getattr(self, field)
        else:
            raise Exception("get(field) : field must be a field name or an integer")

    def get_nparray(self, field):
        if type(field) is int:
            return np.array(getattr(self, self.metadataFormat.dataFields[field]))
        elif type(field) is str:
            return np.array(getattr(self, field))
        else:
            raise Exception("get(field) : field must be a field name or an integer")

    def plot(self, field):
        plt.plot(self.get(field))
        plt.show()


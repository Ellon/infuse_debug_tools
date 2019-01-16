
class MetadataFormat:

    dataFields = []

    def __init__(self):
        pass

    def __repr__(self):
        return "MetadataFormat()"

    def __str__(self):
        res = str(len(self.dataFields)) + " data fields :\n"
        count = 0
        for name in self.dataFields:
            res += str(count) + " : " + name + "\n"
            count += 1
        return res

    def parse_metadata_format_file(self, filename):
 
        formatFile = open(filename, "r")
        
        for line in formatFile:
            self.dataFields.append(line.split()[3])
    

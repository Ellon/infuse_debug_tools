#! /usr/bin/python3

from Metadata import Metadata

metadata = Metadata()

metadata.parse_metadata("data/left_dataformat.txt", "data/left_all_metadata.txt")

metadata.plot(0)


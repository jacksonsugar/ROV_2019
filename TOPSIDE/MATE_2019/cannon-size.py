#!/usr/bin/env python
import numpy as np
import os

# user inputs
skinny_length = input("Skinny length: ")
fat_length = input("Thicc length: ")
r1 = input("Inner Diameter Skinny: ")
r2 = input("Outer Diameter Skinny: ")
r3 = input("Outer Diameter Thicc: ")
density = input("Density: ")

inner_volume = (np.pi*r1*r1*(skinny_length+fat_length))
outer_skinny_volume = (np.pi*r2*r2*skinny_length)
outer_fat_volume = (np.pi*r3*r3*fat_length)

volume = outer_fat_volume + outer_skinny_volume - inner_volume

weight_in_water = volume * density

os.system('clear')

print "Yar mateeee. 'ere art thine outputs:"
print "Volume: %s" % volume
print "Weight in water: %s" % weight_in_water
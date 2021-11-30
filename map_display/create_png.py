################################################################################
# Never Eat Yellow Snow APPS
# 
# Helper command line tool for the map_display app. Used to register maps with point lists.
# This application needs scipy and PIL/pillow installed. It was tested with python 3.2.x but should also
# run on python 2.7.x.
#
# Changelog:
#
# V1.0: Initial version
#
# V1.1: - if possible, use the map png's supplied with ac itself to display the map
#           this needs an initial registering process with the script register_map.py
#       - cars are shown as arrows now (if they move) pointing in the direction of their velocity vector
#       - colors of opponent cars changed according to car state (ahead, behind, overlapped, ...)
#
# V1.4: - use .ini file format instead of .reg.pickled
#
# V1.6: - add possibility (code only) for extended fits
#       - runtime optimizations
#       - add possibility to generate .png files from .pickled files
#       
# V1.9: - add option for specifying the .png size
################################################################################

import sys
import glob
import pickle
import os.path
import traceback
import numpy
import numpy.linalg
import math
from PIL import Image
from PIL import ImageDraw

class Transformer:

    def __init__(self, scale, offsetx, offsety, rotation, aspect_ratio, centerx, centery):
        cr = math.cos(rotation)
        sr = math.sin(rotation)
   
        M1 = numpy.array(
            [[scale, 0, offsetx - centerx + 0.5], 
             [0, scale, offsety - centery + 0.5],
             [0, 0, 1]])
        M2 = numpy.array(
            [[cr, -sr, 0],
             [sr,  cr, 0], 
             [0 ,  0 , 1]])
        M3 = numpy.array(
            [[1, 0, centerx],
             [0, aspect_ratio, centery],
             [0, 0, 1]])
        self.M = numpy.dot(numpy.dot(M3, M2), M1)
        
    def apply(self, points):
        points = numpy.array(points)        
        x = points[:,0]
        y = points[:,2]
        p = numpy.zeros([len(x), 3])
        p[:,0] = x
        p[:,1] = y
        p[:,2] = 1
        return numpy.transpose(numpy.dot(self.M, numpy.transpose(p)))        

def file_path_relative_to_ac(file):
    script_file = os.path.realpath(__file__)
    script_dir = os.path.dirname(script_file)
    ac_dir = os.path.join(script_dir, "..", "..", "..")
    return os.path.relpath(file, ac_dir)

def generate_png_file(pickled_file, map_file, ini_file, linew, track_color, fill_color, png_size):
    points = pickle.load(open(pickled_file, "rb"))
    
    closed_track = False
    dist = numpy.linalg.norm(numpy.array(points[0]) - numpy.array(points[-1]))
    if dist < 30.:
        print ("closed track detected")
        closed_track = True
        points.append(points[0])
    
    minx = min(map(lambda p: p[0], points))
    maxx = max(map(lambda p: p[0], points))
    
    miny = min(map(lambda p: p[2], points))
    maxy = max(map(lambda p: p[2], points))
    
    spreadx = max(maxx-minx, 100)
    spready = max(maxy-miny, 100)
    
    width = png_size
    height = png_size
    
    scale = min((width - 30) / spreadx, (height - 30) / spready)
    # choose offset that avg(minx,maxx) and avg(miny,maxy) is mapped to center
    centerx = 0.5*(minx+maxx)
    centery = 0.5*(miny+maxy)
    
    # scale*centerx + offsetx = width/2 -> offsetx = width/2 - scale*centerx
    offsetx = 0.5*width - scale*centerx
    offsety = 0.5*height - scale*centery
    
    t = Transformer(scale, offsetx, offsety, 0.0, 1.0, 0.5*width, 0.5*height)
    p2d = t.apply(points)
    
    im = Image.new("RGBA", (width, height), (255,255,255,0))
    draw = ImageDraw.Draw(im)
    print("Drawing lines...")
    #print(p2d)
    d0 = -math.floor(linew/2)
    d1 = linew-d0
    
    if closed_track:
        draw.polygon(list(p2d[:,0:2].flat), outline=(255,255,255,0), fill=fill_color)
    
    # the width= argument of draw.line produces ugly results :-) we use another approach
    for dx in numpy.arange(d0, d1, 0.25):
        for dy in numpy.arange(d0, d1, 0.25):
            p2d_c = p2d.copy()
            p2d_c[:,0] += dx
            p2d_c[:,1] += dy
            p2d_c = list(p2d_c[:,0:2].flat)
            draw.line(p2d_c, 
                      fill = track_color)
    
    del draw
    print("Saving image ...")
    im.save(map_file, "PNG")
    print("Saving ini file ...")
    save({'scale':scale, 
          'offsetx':offsetx, 
          'offsety':offsety, 
          'map_size_x':width, 
          'map_size_y':height}, 
         ini_file)
    
def save(register_info, out_file):
    scale = register_info['scale']
    ox = register_info['offsetx']
    oy = register_info['offsety']
    register_info['scale_inv'] = 1./float(scale)
    register_info['offsetx_scaled'] = float(ox)*1./float(scale)
    register_info['offsety_scaled'] = float(oy)*1./float(scale)
    
    open(out_file, 'w').write("""
[PARAMETERS]
WIDTH=%(map_size_x)f
HEIGHT=%(map_size_y)f
MARGIN=20
SCALE_FACTOR=%(scale_inv)f
MAX_SIZE=1600
X_OFFSET=%(offsetx_scaled)f
Z_OFFSET=%(offsety_scaled)f
DRAWING_SIZE=10
""" % register_info )

    print(register_info)
        
def hexcolor2tuple(cstr):
    assert(len(cstr) == 2*4+2)
    assert(cstr[0:2] == "0x")
    res = (
        int(cstr[2:4],base=16),
        int(cstr[4:6],base=16),
        int(cstr[6:8],base=16),
        int(cstr[8:10],base=16)
        )
    return res
    
if __name__ == "__main__":
    png_size = 512
    if sys.argv[1] == "-s":
        png_size = int(sys.argv[2])
    sys.argv = [sys.argv[0]] + sys.argv[3:]
    if len(sys.argv) == 6 and sys.argv[1] == "-p":
        pickled_file = sys.argv[2]
        if '*' in pickled_file:
            pickled_files = glob.glob(pickled_file)
        else:
            pickled_files = [pickled_file]
        for pickled_file in pickled_files:
            map_file = os.path.splitext(pickled_file)[0] + ".png"
            ini_file = os.path.splitext(pickled_file)[0] + ".ini"
            track_width = float(sys.argv[3])
            track_color = hexcolor2tuple(sys.argv[4])
            fill_color = hexcolor2tuple(sys.argv[5])
            generate_png_file(pickled_file, map_file, ini_file, track_width, track_color, fill_color, png_size)
    else:
        print("""Usage: 
create_png.py [-s <png_size_px>] -p <pickled_file> <track_width_px> <track_color_hex> <fill_color_hex>:
        Create .png and .ini files from the supplied .pickled file.
        <pickled_file> the pickled file containing the track 
            (if it contains a '*', then a globbing scheme will be applied)
        <track_width_px> is the track width in pixel
        <track_color_hex> is the hex color code (see below) of the track
        <fill_color_hex> is the hex color code (see below) of the inside of the track 
            (meaningful only for closed tracks)
        color codes are supplied in the format 0xRRGGBBAA, e.g:
            0xff0000ff: opaque red color
            0x00000000: full transparent black (no color)
            0xacacacc0: mid gray, could be used for track color
            0x00ff0016: transparent green, could be used for fill color
        <png_size_px> is the width and height of the created png file (default 512)
        """)
        sys.exit(1)
    
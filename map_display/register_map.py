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
################################################################################

import sys
import glob
import pickle
import os.path
import traceback
import matplotlib.pyplot as plt
import scipy.optimize as opt
from scipy import misc
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

def score(points, dt, scale, offsetx, offsety, rotation, aspect_ratio):
    dmax = numpy.max(points,0)
    dmin = numpy.min(points,0)
    w = numpy.size(dt,1)
    h = numpy.size(dt,0)
    scale0 = min(w/(dmax[0]-dmin[0]), h/(dmax[2]-dmin[2]))
    minscale = scale0/2
    maxscale = scale0*2
    if scale < minscale or scale > maxscale or aspect_ratio < 0.5 or aspect_ratio > 1.5:
        res = 1e10
    else:
        res = 0
    maxdt = max(dt.flat)
    
    trans = Transformer(scale, offsetx, offsety, rotation, aspect_ratio, numpy.size(dt,1)/2., numpy.size(dt,0)/2.)
    tp = trans.apply(points)
    ix = tp[:,0]
    iy = tp[:,1]
    xinrange = numpy.logical_and(ix >= 0, ix < numpy.size(dt,1)-1)
    yinrange = numpy.logical_and(iy >= 0, iy < numpy.size(dt,0)-1)
    bothok = numpy.logical_and(xinrange, yinrange)
    ix = ix[bothok]
    iy = iy[bothok]
    ul = dt.flat[(iy.astype(numpy.int32)  )*numpy.size(dt,1)+ix.astype(numpy.int32)  ]
    ur = dt.flat[(iy.astype(numpy.int32)  )*numpy.size(dt,1)+ix.astype(numpy.int32)+1]
    bl = dt.flat[(iy.astype(numpy.int32)+1)*numpy.size(dt,1)+ix.astype(numpy.int32)  ]
    br = dt.flat[(iy.astype(numpy.int32)+1)*numpy.size(dt,1)+ix.astype(numpy.int32)+1]
    dy = iy - numpy.floor(iy)
    dx = ix - numpy.floor(ix)
    v = ((ur*(dx) + ul*(1-dx))*(1-dy)+
         (br*(dx) + bl*(1-dx))*(dy)  )
    res = numpy.sum(v**2)
    res += numpy.sum(numpy.logical_not(bothok))*(maxdt**2)
    print("score[%.5f, %.2f, %.2f, %.2f, %.2f]=%e" % (scale, offsetx, offsety, rotation*180/math.pi, aspect_ratio, res))
    return res

def distance_transform(binimg):
    h = numpy.size(binimg, 0)
    w = numpy.size(binimg, 1)
    maxd = h+w
    res = numpy.zeros((h, w))
    res[:,:] = maxd
    res[binimg] = 0
    
    for y in range(h):
        for x in range(w):
            if x > 0:
                res[y,x] = min(res[y, x-1] + 1, res[y,x])
            if y > 0:
                res[y,x] = min(res[y-1, x] + 1, res[y,x])
    
    for y in range(h-1,-1,-1):
        for x in range(w-1,-1,-1):
            if x < w-1:
                res[y,x] = min(res[y, x+1] + 1, res[y,x])
            if y < h-1:
                res[y,x] = min(res[y+1, x] + 1, res[y,x])
    return res

extended_parameters = 0
    
def register_map(points,map_img):
    
    dt = distance_transform(map_img[:,:,3] > 128)
    
    points = numpy.array(points)
    if extended_parameters:
        opt_fun = lambda x, points, map_img: score(points, map_img, x[0], x[1], x[2], x[3], x[4])
    else:
        opt_fun = lambda x, points, map_img: score(points, map_img, x[0], x[1], x[2], 0, 1.0)
        
    dmax = numpy.max(points,0)
    dmin = numpy.min(points,0)
    w = numpy.size(map_img,1)
    h = numpy.size(map_img,0)
    scale0 = min(w/(dmax[0]-dmin[0]), h/(dmax[2]-dmin[2]))
    # we have a mapping s*c + o 
    #  (where c is the world coordinate, s the scale factor and o the offsetx)
    # we try out different start values for o:
    # s*minc + o1 = 0   (aligns the minimum coordinate to the left/top map
    # s*maxc + o2 = w/h (aligns the maximum coordinate to the right/bottom map
    # o3 = (o1+o2)/2    (aligns to the center
    o1x = -scale0*dmin[0]
    o1y = -scale0*dmin[2]
    o2x = w - scale0*dmax[0]
    o2y = h - scale0*dmax[2]
    o3x = (o1x+o2x)*0.5
    o3y = (o1y+o2y)*0.5
    bestScore = None
    bestStartOffset = None
    for o in [(o1x, o1y),
              (o1x, o2y),
              (o1x, o3y),
              (o2x, o1y),
              (o2x, o2y),
              (o2x, o3y),
              (o3x, o1y),
              (o3x, o2y),
              (o3x, o3y)]:
        s = score(points, dt, scale0, o[0], o[1], 0, 1.)
        if bestScore is None or s < bestScore:
            bestScore = s
            bestStartOffset = o
    offsetx0 = bestStartOffset[0]
    offsety0 = bestStartOffset[1]
    
    print("\n")
    if extended_parameters:
        x0 = [scale0, offsetx0, offsety0, 0, 1]
    else:
        x0 = [scale0, offsetx0, offsety0]
    xo = opt.fmin(opt_fun, x0, (points, dt))
    scale = xo[0]
    offsetx = xo[1]
    offsety = xo[2]
    if extended_parameters:
        rotation = xo[3]
        aspect_ratio = xo[4]
    else:
        rotation = 0
        aspect_ratio = 1.
        
    transformer = Transformer(scale, offsetx, offsety, rotation, aspect_ratio, numpy.size(dt,1)/2., numpy.size(dt,0)/2.)
    tp = transformer.apply(points)
    for p in tp:
        ix = p[0]
        iy = p[1]
        if ix >= 0 and ix < numpy.size(map_img,1) and iy >= 0 and iy < numpy.size(map_img,0):
            map_img[iy, ix, :] = [255, 0, 0, 255]
    plt.imshow(map_img)
    #plt.imshow(dt)
    plt.show()
    
    return scale, offsetx, offsety

def file_path_relative_to_ac(file):
    script_file = os.path.realpath(__file__)
    script_dir = os.path.dirname(script_file)
    ac_dir = os.path.join(script_dir, "..", "..", "..")
    return os.path.relpath(file, ac_dir)

def process(pickled_file, map_file, out_file):
    map_img = misc.imread(map_file)
    points = pickle.load(open(pickled_file, "rb"))
    
    scale, ox, oy = register_map(points, map_img)
    
    register_info = {
        'scale': float(scale), 
        'offsetx': float(ox), 
        'offsety': float(oy), 
        'map': file_path_relative_to_ac(map_file),
        'map_size_x' : float(numpy.size(map_img, 1)),
        'map_size_y' : float(numpy.size(map_img, 0)),
    }
    save(register_info, out_file)

def generate_png_file(pickled_file, map_file, ini_file, linew, track_color, fill_color):
    points = pickle.load(open(pickled_file, "rb"))
    
    closed_track = False
    dist = numpy.linalg.norm(numpy.array(points[0]) - numpy.array(points[-1]))
    if dist < 30.:
        print ("closed track detected")
        closed_track = True
        points.append(points[0])
    
    minx = min(map(lambda p: p[0], points)) - 20.
    maxx = max(map(lambda p: p[0], points)) + 20.
    
    miny = min(map(lambda p: p[2], points)) - 20.
    maxy = max(map(lambda p: p[2], points)) + 20.
    
    spreadx = max(maxx-minx, 100)
    spready = max(maxy-miny, 100)
    
    width = 512
    height = 512
    
    scale = min(width / spreadx, height / spready)
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
    if len(sys.argv) == 2 and sys.argv[1] == "-a":
        os.chdir("data")
        for f in glob.glob("*.pickled"):
            track = f[:-len(".pickled")]
            pickled_file = track + ".pickled"
            map_file = "../../../../content/tracks/"+ track + r"/map.png"
            out_file = track + ".ini"
            print("Processing", track, f)
            try:
                process(pickled_file, map_file, out_file)
            except: 
                print( "Error while processing", f )
                print( traceback.format_exc() )
                print()
    elif len(sys.argv) == 2 and sys.argv[1] == "-c":
        os.chdir("data")
        for f in glob.glob("*.reg.pickled"):
            reginfo = pickle.load(open(f, "rb"))
            save(reginfo, f[:-len("reg.pickled")] + "ini")
    elif len(sys.argv) == 6 and sys.argv[1] == "-p":
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
            generate_png_file(pickled_file, map_file, ini_file, track_width, track_color, fill_color)
    elif len(sys.argv) == 4:
        process(sys.argv[1], sys.argv[2], sys.argv[3])
    else:
        print("""Usage: 
register_map.py -a:
        Register all .pickled files in data/ to the map.png files 
        supplied by the track. Creates corresponding map.ini files in data/
register_map.py -c:
        Convert all (old) .reg.pickled files in data/ into map.ini format
register_map.py -p <pickled_file> <track_width_m> <track_color_hex> <fill_color_hex>:
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
            0xdcdcdcc0: bright gray, could be used for track color
            0x00ff0032: transparent green, could be used for fill color
register_map.py <pickled_file> <png_file> <map_ini_out>
        Register the specified pickled file to the specified png file. Result is 
        written to map_ini_out.
        """)
        sys.exit(1)
    
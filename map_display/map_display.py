################################################################################
# Never Eat Yellow Snow APPS
# V1.14, 25.05.2017, AC 1.14.2
# 
# App for displaying track maps without ridiculous zoom factors.
# Tracks must be teached by driving a whole clean lap (best in hotlap mode) until the
# map display shows up. They are saved beneath the script itself. Optionally, these
# recorded tracks may be registered with a map.png file from assetto corsa to 
# provide better graphical display of the maps (see register_map.py)
#
# Credits:
#   www.aha-soft.com for the icon set used
#
# TODO:
#
# - opponent coloring in qualy and training according to out-lap or normal
# - configurable zoom (implemented but disabled currently)
# - "gps track recording mode" (snjper)
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
# V1.2: - app-icon
#       - transparent title bar (using hacks from snjper, put link to icon hack in initial post)
#       - try using TyreHeadingVector instead of velocity vector (seems that the API is not yet implemented, so fall back using velocity is used)
#       - move pickled files to data subdirectory
#       - opponent coloring in qualifying and training, for the moment, the color is constantly yellow (instead of random before)
#       - development mode (prevent track recording in user mode to ensure that the .pickled files are sane, see development_mode)
#       - alpha rendering for car arrows (see alphaCarArrows variable)
#
# V1.3: - fix session type changes and resets (if possible)
#       - prefer png's in data folder with same name then the pickle file over the ones in the content folder
#       - introduce workarounds.py for implementing workarounds around issues in current ac python api
#       - add profiling info in py_log.txt file
#
# V1.4: - read the track's map.ini file to get the necessary parameters for displaying (now every track supported by the builtin app should be supported by this app as well)
#       - the .reg.pickled files in data/ are obsolete now and they are not read anymore (they have been removed)
#       - interactive toggling between different versions of map displays (using content/tracks/<trackname>/map*.png and apps/python/map_display/data/<trackname>*.png and apps/python/map_display/data/<trackname>*.pickled)
#       - support different color schemes for coloring the opponents (two by now), interactive toggling
#       - implemented but disabled due to severe issues in the python api: interactive zooming
#       - fix possible ac issues when the acMain function throws an exception
#       - gui mode for recording track maps (there is a record, discard and save button)
#       - display the so-far recorded track
#       - possibility for A-B tracks in point list (.pickled) mode, draw the closing line only when reasonably close
#       
# V1.5: - just a fix for the icon paths
# 
# V1.6: - add some user feedback in the title bar (so the user can see what's going on)
#       - optimizations in the track display (when using a .pickled / point list file). The ac limit on the number of points shall be reached much later now (nord_snoopy works ok).
#       - save user settings in map_display.ini file
#       - when a recording is saved, it is immediately used (no need to click lower left icon)
#       - fix possibility to use vallelunga-club map in vallelunga and silverstone-international map in silverstone
#       - new icons from Wattie (thanks a lot!)
#       - eyecandy versions of maps for built-in tracks (thanks snjper for the idea with the green inside!)
#       - register_map.py: add possibility (code only) for extended fits
#       - register_map.py: runtime optimizations
#       - register_map.py: add possibility to generate .png files from .pickled files
#       
# V1.7: - compatibility with AC 0.20
# 
# V1.8: - automatic .png generation when a track is recorded
#
# V1.9: - add more general options in the .ini file
#       - adjustable png sizes
#       - better responses when recordings are started or saved
#       - use packages as a namespace for required modules, avoiding side-effects from other apps
#
# V1.10: - update for AC 0.22
#        - color scheme is now restored correctly from ini file (saved between runs)
# 
# V1.11: - experimental workarounds for issues observed with AC 0.22
#        - handling of nurburgring-sprint
# V1.12: - handling of silverstone-national
#        - fix "dangling" cars of AC 1.0RC
# V1.13: - another workaround for dangling cars in MP (after a player has disconnected)
#        - support the track configurations introduced in AC 1.1
#        - improve invariance against influences from other apps (e.g., older versions of helicorsa)
# V1.14: - AC 1.3 / 64 bit compatibility
# V1.15: - respect the new focusedCar() API to change the perspective to the currently spectated car
#        - new coloring scheme for non-race sessions (based on out-lap information)
#        - fix very high CPU usage when recording maps of long tracks
#        - changed default resolution of generated maps to 2048 x 2048 pixel
#        - fix display red line when recording a map (is not cut anymore)
# V1.16: - potential fix for the disappearing app issue reported by many users. Thanks to RD user @AdderSwim
#        - add arrow_size_factor to .ini file
################################################################################
import sys
import os
import platform
if platform.architecture()[0] == "64bit":
    sysdir='apps/python/map_display/map_display_lib/stdlib64'
else:
    sysdir='apps/python/map_display/map_display_lib/stdlib'
sys.path.insert(0, sysdir)
os.environ['PATH'] = os.environ['PATH'] + ";."
import configparser
import math
import traceback
import pickle
import os.path
import time
import re
import glob
import subprocess

import ac
ac.log("python platform.architecture="+str(platform.architecture())+"; sys.version="+str(sys.version))
import acsys
try:
    from map_display_lib import workarounds
    import map_display_lib
except:
    ac.log("map_display.py: error while importing map_display_lib:")
    ac.log(traceback.format_exc())

try:

    config = configparser.ConfigParser()
    config.read("apps/python/map_display/map_display.ini")

    use_transparent_hacks = 1
    x_app_size = config.getint("WINDOW", "app_width", fallback=300)
    y_app_size = config.getint("WINDOW", "app_height", fallback=325)
    top_space = config.getint("WINDOW", "app_border_top", fallback=30)
    bottom_space = config.getint("WINDOW", "app_border_bottom", fallback=5)
    left_space = config.getint("WINDOW", "app_border_left", fallback=5)
    right_space = config.getint("WINDOW", "app_border_right", fallback=5)
    background_opacity = config.getfloat("WINDOW", "background_opacity", fallback=0.0)    

    zoom_enabled = 0
    verbosity = 2

    alphaCarArrows = 1.0 # better leave it as 1, otherwise it gets difficult to recognize the arrows

    colorSchemes = [
        { # lfs
            'colorEgo' : [0.0, 0.8, 0.0, alphaCarArrows],
            'colorOppBehindSameLap' : [217/255., 140/255., 36/255., alphaCarArrows],
            'colorOppBehindOverlapped' : [111/255., 71/255., 18/255., alphaCarArrows],
            'colorOppAheadSameLap' : [231/255., 231/255., 16/255., alphaCarArrows],
            'colorOppAheadOverlapped' : [231/255., 231/255., 159/255., alphaCarArrows],
            'colorOppUnknown' : [231/255., 231/255., 16/255., alphaCarArrows],
        },
        { # high contrast
            'colorEgo' : [0.0, 0.8, 0.0, alphaCarArrows],
            'colorOppBehindSameLap' : [217/255., 140/255., 36/255., alphaCarArrows],
            'colorOppBehindOverlapped' : [0.8, 0.2, 0.2, alphaCarArrows],
            'colorOppAheadSameLap' : [231/255., 231/255., 16/255., alphaCarArrows],
            'colorOppAheadOverlapped' : [0.2, 0.2, 0.8, alphaCarArrows],
            'colorOppUnknown' : [231/255., 231/255., 16/255., alphaCarArrows],
        },
    ]
    for i,s in enumerate(["COLOR_SCHEME_1", "COLOR_SCHEME_2"]):
        for k in colorSchemes[i].keys():
            colorSchemes[i][k][0] = config.getint(s, k + "_Red", fallback = int(colorSchemes[i][k][0]*255.+0.5))/255.
            colorSchemes[i][k][1] = config.getint(s, k + "_Green", fallback = int(colorSchemes[i][k][1]*255.+0.5))/255.
            colorSchemes[i][k][2] = config.getint(s, k + "_Blue", fallback = int(colorSchemes[i][k][2]*255.+0.5))/255.
            colorSchemes[i][k][3] = config.getint(s, k + "_Alpha", fallback = int(colorSchemes[i][k][3]*255.+0.5))/255.

    png_size = config.getint("PNG_GENERATION", "size", fallback = 2048)
    png_line_width = config.getint("PNG_GENERATION", "line_width", fallback = 3)
    png_line_color = (
        config.getint("PNG_GENERATION", "line_color_red", fallback=0xac),
        config.getint("PNG_GENERATION", "line_color_green", fallback=0xac),
        config.getint("PNG_GENERATION", "line_color_blue", fallback=0xac),
        config.getint("PNG_GENERATION", "line_color_alpha", fallback=0xc0),
    )
    png_fill_color = (
        config.getint("PNG_GENERATION", "fill_color_red", fallback=0x00),
        config.getint("PNG_GENERATION", "fill_color_green", fallback=0xff),
        config.getint("PNG_GENERATION", "fill_color_blue", fallback=0x00),
        config.getint("PNG_GENERATION", "fill_color_alpha", fallback=0x16),
    )
except:
    ac.log(traceback.format_exc())
    
def acdebug(msg):
    if verbosity >= 3:
        ac.log("map_display.py[DEBUG]: %s" % msg)

def acinfo(msg):
    if verbosity >= 2:
        ac.log("map_display.py[INFO]: %s" % msg)
    
def acwarning(msg):
    if verbosity >= 1:
        ac.log("map_display.py[WARNING]: %s" % msg)

def acerror(msg):
    ac.log("map_display.py[ERROR]: %s" % msg)

def acCreateButton(app, label, posx, posy, sizex, sizey, onClickCallback, bg_tex):
    btn = ac.addButton(app, label)
    ac.setPosition(btn, posx, posy);
    ac.addOnClickedListener(btn, onClickCallback)
    ac.setBackgroundTexture(btn, bg_tex)
    ac.drawBackground(btn, 0)
    ac.drawBorder(btn, 0)
    ac.setSize(btn, sizex, sizey)
    ac.setVisible(btn, False)
    return btn
    
def acCreateLabel(app, label, posx, posy, sizex, sizey):
    label = ac.addLabel(app, label)
    ac.setPosition(label, posx, posy)
    ac.setSize(label, sizex, sizey)
    ac.drawBackground(label, 0)
    ac.drawBorder(label, 0)
    ac.setVisible(label, False)
    return label

def distance(p1, p2):
    dp = [p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]]
    d = math.sqrt(dp[0]**2 + dp[1]**2 + dp[2]**2)
    return d

class CarWrapper:
    def __init__(self, carId):
        self.carId = carId
        self.lastLapCount = ac.getCarState(self.carId, acsys.CS.LapCount)
        self.out_lap = False
        
def content_path(track_name, track_config, postfix):
    if not track_config is None:
        return "content\\tracks\\%s\\%s\\%s" % (track_name, track_config, postfix)
    return "content\\tracks\\%s\\%s" % (track_name, postfix)
    
class MapPngDisplay:

    def __init__(self, parent, pngFile, track_name, track_config):
        self.parent = parent
        self.pngFile = pngFile
        # search for an ini file for this png file
        # first priority: an .ini file with the same prefix than the .png file (e.g., path/imola.png -> path/imola.ini)
        # second priority: the default .ini file from the track
        iniFile = os.path.splitext(pngFile)[0] + ".ini"
        
        if not os.path.exists(iniFile):
            iniFile = content_path(track_name, track_config, "data\\map.ini")
        acinfo("Associating ini file %s to png file %s" % (iniFile, pngFile))
        self.register_info = self.parse_map_ini(iniFile)
        # make sure the transformation is set up correctly
        self.getTransformation()
        self.map_texture = None
        
    def getTransformation(self):
        if not hasattr(self, "scale"):
            class Transformation:
                def __init__(self, s, ox, oy):
                    self.scale = s
                    self.offsetx = ox
                    self.offsety = oy
                def __str__(self):
                    return "(%.3f, %.1f, %.1f)" %(self.scale,self.offsetx,self.offsety)
                    
            point2map = Transformation(self.register_info['scale'], self.register_info['offsetx'], self.register_info['offsety'])
            maxx = self.register_info['map_size_x']
            maxz = self.register_info['map_size_y']
            scalex = (x_app_size-left_space-right_space)/maxx
            scaley = (y_app_size-top_space-bottom_space)/maxz
            scale = min(scalex, scaley)
            offsetx = left_space
            offsety = top_space
            offsetx += ((x_app_size-right_space)-(maxx*scale+offsetx))*0.5
            offsety += ((y_app_size-bottom_space)-(maxz*scale+offsety))*0.5
            map2screen = Transformation(scale, offsetx, offsety)
            # mapping a point p_w to screen is done by the following
            # 1. map it to the map:    p_m = p_w*point2map.scale + point2map.offset
            # 2. map it to the screen: p_s = p_m*map2screen.scale + map2screen.offset
            point2screen = Transformation(map2screen.scale*point2map.scale,
                                          point2map.offsetx*map2screen.scale + map2screen.offsetx,
                                          point2map.offsety*map2screen.scale + map2screen.offsety)
            self.scale = point2screen.scale
            self.offsetx = point2screen.offsetx
            self.offsety = point2screen.offsety        
            self.map2screen = map2screen
            self.point2map = point2map
            acdebug("map2screen = %s; point2map = %s" % (str(self.map2screen), str(self.point2map)))
            
        return (self.scale, self.offsetx, self.offsety)
    
    def render(self):
        if self.map_texture is None:
            self.map_texture = ac.newTexture(self.pngFile)
            acinfo("Loaded texture %s (%s)" % (self.pngFile, str(self.map_texture)))
        self.parent.glQuadTextured(self.map2screen.offsetx, 
                                   self.map2screen.offsety, 
                                   self.register_info['map_size_x']*self.map2screen.scale, 
                                   self.register_info['map_size_y']*self.map2screen.scale,
                                   self.map_texture)
    
    def parse_map_ini(self, track_ini_file):
        acdebug("parsing %s" % track_ini_file)
        values = {}
        regexps = {}
        keys = ["WIDTH","HEIGHT","SCALE_FACTOR","X_OFFSET","Z_OFFSET"]
        for k in keys:
            regexps[k] = re.compile(k+r"\s*=\s*(-?[0-9]+([.][0-9]*){0,1})(\s*;.*){0,1}")
        n = 0
        for l in open(track_ini_file,"r").readlines():
            l = l.strip()
            if len(l) == 0: continue
            if l[0] == ";": continue
            for k in keys:
                m = regexps[k].match(l)
                if not m is None:
                    values[k] = float(m.group(1))
            n += 1
            if n > 50:
                break
        res = {}
        res['scale'] = 1./values["SCALE_FACTOR"]
        res['offsetx'] = values["X_OFFSET"]/values["SCALE_FACTOR"]
        res['offsety'] = values["Z_OFFSET"]/values["SCALE_FACTOR"]
        res['map_size_x'] = values["WIDTH"]
        res['map_size_y'] = values["HEIGHT"]
        acinfo("result of map.ini parsing: " + str(res))
        return res
        
class MapPointListDisplay:
    
    def __init__(self, parent, pointList, color = [0.8, 0.8, 0.8]):
        self.parent = parent
        self.color = color
        if type(pointList) == type(""):
            self.points = pickle.load(open(pointList, "rb"))
            acinfo("Loaded point list numPoints=%d" % len(pointList))
            # assert that points is a list of 3 dimensional tuples
            for p in self.points:
                assert len(p) == 3
        else:
            self.points = pointList
        self.mapped_p = []
        self.closed = False
        self.maxCompressedPoints = 40
        self.maxCompressDist = 0.5 # 1 pixel
        self.maxNumPointsPerPrimitive = 30
        
    def mergePoints(self, pointList):
        self.points.extend(pointList[len(self.points):])
        
    def getTransformation(self):
        if not hasattr(self, "scale"):
            minx = min(map(lambda p: p[0], self.points))
            maxx = max(map(lambda p: p[0], self.points))
            minz = min(map(lambda p: p[2], self.points))
            maxz = max(map(lambda p: p[2], self.points))
            if maxx - minx < 100.:
                maxx += 50.
                minx -= 50.
            if maxz - minz < 100.:
                maxz += 50.
                minz -= 50.
            scalex = (x_app_size-left_space-right_space)/(maxx-minx)
            scaley = (y_app_size-top_space-bottom_space)/(maxz-minz)
            
            scale = min(scalex, scaley)
            offsetx = -minx*scale+left_space
            offsety = -minz*scale+top_space
            offsetx += ((x_app_size-right_space)-(maxx*scale+offsetx))*0.5
            offsety += ((y_app_size-bottom_space)-(maxz*scale+offsety))*0.5
            
            self.setTransformation(scale, offsetx, offsety)
        return (self.scale, self.offsetx, self.offsety)
        
    def compress(self, points, delta_i):
        res = [(points[0][0], points[0][1], delta_i)]
        n = len(points)
        li = 0
        for i in range(1,n):
            p = points[i]
            lp = points[li]
            
            if i - li < self.maxCompressedPoints:
                pointsInLine = True
                # line (x0, y0) -> (x1, y1)
                # d = (dx, dy) = (x1-x0, y1-y0)
                
                # r = (rx, ry) = d/sqrt(dx^2 + dy^2)
                # n = (nx, ny) = (-ry, rx)
                
                # -ry*x + rx*y + q = 0
                # q = ry*x0 - rx*y0
                #
                # dist(line01, p) = -ry*px+rx*py+ry*x0-rx*y0
                
                dx = p[0] - lp[0]
                dy = p[1] - lp[1]
                nd = 1./math.sqrt(dx**2 + dy**2)
                rx = dx*nd
                ry = dy*nd
                
                for j in range(li+1, i):
                    q = points[j]
                    dist = abs(-ry*q[0]+rx*q[1]+ry*p[0]-rx*p[1])
                    if dist > self.maxCompressDist:
                        pointsInLine = False
            else:
                pointsInLine = False
                    
            if i == n-1 or not pointsInLine:
                res.append((p[0], p[1], delta_i+i))
                li = i
        return res
        
    def map_point(self, p):
        return (p[0]*self.scale+self.offsetx, p[2]*self.scale+self.offsety)
        
    def setTransformation(self, scale, offsetx, offsety):
        if (scale != getattr(self,"scale",scale-1.) or 
            offsetx != getattr(self, "offsetx", offsetx-1.) or 
            offsety != getattr(self, "offsety", offsety-1) ):
            
            self.scale = scale
            self.offsetx = offsetx
            self.offsety = offsety

            self.mapped_p = []

        np = 0 if len(self.mapped_p) == 0 else self.mapped_p[-1][2]
        if len(self.points) > np:
            if len(self.mapped_p) < 2:
                iStart = 0
                self.mapped_p = []
            else:
                iStart = self.mapped_p[-2][2]
                self.mapped_p = self.mapped_p[:-2]
        
            map_p = [self.map_point(p) for p in self.points[iStart:]]
            self.mapped_p.extend(self.compress(map_p, iStart))
                
            self.closed = distance(self.points[0], self.points[-1]) <= 2.*self.parent.delta_s
    
    def render(self):
        ac.glBegin(acsys.GL.LineStrip)
        ac.glColor3f(*self.color)
        cnt = 0
        for p in self.mapped_p:
            self.parent.glVertex2f(p[0], p[1])
            cnt += 1
            if cnt >= self.maxNumPointsPerPrimitive:
                cnt = 0
                ac.glEnd()
                ac.glBegin(acsys.GL.LineStrip)
                self.parent.glVertex2f(p[0], p[1])
        if self.closed:
            self.parent.glVertex2f(self.mapped_p[0][0], self.mapped_p[0][1])
        ac.glEnd()

class MapDisplay:
    
    def __init__(self, app):
        self.BEHIND_SAME_LAP = 0
        self.BEHIND_OVERLAPPED = 1
        self.AHEAD_SAME_LAP = 2
        self.AHEAD_OVERLAPPED = 3
        
        self.UNKNOWN_OPP_STATE = 6
        
        self.oppColors = {
            self.BEHIND_SAME_LAP : colorSchemes[0]['colorOppBehindSameLap'],
            self.BEHIND_OVERLAPPED : colorSchemes[0]['colorOppBehindOverlapped'],
            self.AHEAD_SAME_LAP : colorSchemes[0]['colorOppAheadSameLap'],
            self.AHEAD_OVERLAPPED : colorSchemes[0]['colorOppAheadOverlapped'],
            self.UNKNOWN_OPP_STATE : colorSchemes[0]['colorOppUnknown'],
        }
        self.delta_s = 10.
        self.track_name = ac.getTrackName(0)
        self.track_config = ac.getTrackConfiguration(0)
        if self.track_config == "":
            self.track_config = None
            self.md_data_prefix = self.track_name
        else:
            self.md_data_prefix = self.track_name + "-" + self.track_config
        self.carWrappers = {}

        self.maps = []
        self.map_displays = {}
        pointLists = glob.glob(r"apps\\python\\map_display\\data\\%s*.pickled" % self.md_data_prefix)
        acdebug(str(pointLists))
        for p in pointLists[:]:
            if os.path.split(p)[-1].lower().find(".reg.") != -1:
                pointLists.remove(p)
        
        potential_maps = (
            [content_path(self.track_name, self.track_config, "map.png")] +
            glob.glob(content_path(self.track_name, self.track_config, "map*.png")) +
            glob.glob(r"apps\\python\\map_display\\data\\%s*.png" % self.md_data_prefix) +
            pointLists
            )
            
        clash_names = ["silverstone-international", "silverstone-national", "vallelunga-club", "nurburgring-sprint"]
        try:
            clash_names.remove(self.md_data_prefix)
        except ValueError:
            pass
        
        acdebug(potential_maps)
        for f in potential_maps:
            f = os.path.normpath(f)
            filename = os.path.split(f)[1]
            clash = False
            for c in clash_names:
                if filename.find(c) == 0:
                    clash = True
            if clash:
                continue
            if not f in self.maps and os.path.exists(f):
                try:
                    if os.path.splitext(f)[1].lower() == ".pickled":
                        display = MapPointListDisplay(self, f)
                    else:
                        display = MapPngDisplay(self, f, self.track_name, self.track_config)
                    self.maps.append(f)                
                    self.map_displays[f] = display
                except:
                    acerror("Error loading %s (discarding)" % f)
                    acerror(traceback.format_exc())
        
        acinfo("found %d maps" % len(self.maps))

        self.currentMapIdx = 0
        try:
            mapFile = config.get(self.md_data_prefix, "map_file", fallback="")
            self.currentMapIdx = self.maps.index(mapFile)
        except:
            pass
        self.currentColorScheme = config.getint("GLOBAL", "colorScheme", fallback=0) % 2
        self.oppColors = {
            self.BEHIND_SAME_LAP : colorSchemes[self.currentColorScheme]['colorOppBehindSameLap'],
            self.BEHIND_OVERLAPPED : colorSchemes[self.currentColorScheme]['colorOppBehindOverlapped'],
            self.AHEAD_SAME_LAP : colorSchemes[self.currentColorScheme]['colorOppAheadSameLap'],
            self.AHEAD_OVERLAPPED : colorSchemes[self.currentColorScheme]['colorOppAheadOverlapped'],
            self.UNKNOWN_OPP_STATE : colorSchemes[self.currentColorScheme]['colorOppUnknown'],
        }        
        self.recording = False
        self.recordedPoints = []
        
        self.zoom = config.getfloat("GLOBAL", "zoom", fallback=1.0)
        self.coffsetx = 0.
        self.coffsety = 0.
        self.arrow_size = config.getfloat("GLOBAL", "arrow_size_factor", fallback=1.0)
        
        self.interactiveMode = False
        self.buttons = []
        
        self.image_creation_processes = {}
        
        self.recDisplay = None
        
        if zoom_enabled:
            self.buttons.append( acCreateButton(app, "",  5,  5, 20, 20, on_click_zoom_in , r"apps\python\map_display\icons\zoom_in.png") )
            self.buttons.append( acCreateButton(app, "",  5, 30, 20, 20, on_click_zoom_out, r"apps\python\map_display\icons\zoom_out.png") )
            self.buttons.append( acCreateButton(app, "",  5, 55, 20, 20, on_click_zoom_def, r"apps\python\map_display\icons\zoom_def.png") )

        self.buttons.append( acCreateButton(app, "",  5, y_app_size-25, 20, 20, on_click_change_map  , r"apps\python\map_display\icons\toggle_map.png") )
        self.buttons.append( acCreateButton(app, "",  5, y_app_size-50, 20, 20, on_click_color_scheme, r"apps\python\map_display\icons\color_scheme.png") )
        self.buttons.append( acCreateButton(app, "", x_app_size-25, top_space +  0, 20, 20, on_click_start_rec   , r"apps\python\map_display\icons\start_rec.png") )
        self.btnRec = self.buttons[-1]
        self.buttons.append( acCreateButton(app, "", x_app_size-25, top_space + 25, 20, 20, on_click_discard_rec , r"apps\python\map_display\icons\discard_rec.png") )
        self.buttons.append( acCreateButton(app, "", x_app_size-25, top_space + 50, 20, 20, on_click_save_rec    , r"apps\python\map_display\icons\save_rec.png") )
        
        # Hmm. when setting the label visible, the rendered track disappears. We use title instead ...
        #self.statusBar = acCreateLabel(app, "", 30, 0, x_app_size - 35, 15)
        #self.buttons.append( self.statusBar )
        
    def close(self):
        acinfo("writing ini file")
        config["GLOBAL"] = {
            'colorScheme': str(self.currentColorScheme),
            'zoom': str(self.zoom),
            'arrow_size_factor': str(self.arrow_size),
        }
        config[self.md_data_prefix] = {
            'map_file' : self.maps[self.currentMapIdx],
        }
        if not "WINDOW" in config: config["WINDOW"] = {}
        config["WINDOW"]["app_width"] = str(x_app_size)
        config["WINDOW"]["app_height"] = str(y_app_size)
        config["WINDOW"]["app_border_top"] = str(top_space)
        config["WINDOW"]["app_border_bottom"] = str(bottom_space)
        config["WINDOW"]["app_border_left"] = str(left_space)
        config["WINDOW"]["app_border_right"] = str(right_space)
        config["WINDOW"]["background_opacity"] = str(background_opacity)
        
        for i,s in enumerate(["COLOR_SCHEME_1", "COLOR_SCHEME_2"]):
            if not s in config: config[s] = {}
            for k in colorSchemes[i].keys():
                config[s][k + "_Red"]   = str(int(colorSchemes[i][k][0]*255.+0.5) )
                config[s][k + "_Green"] = str(int(colorSchemes[i][k][1]*255.+0.5) )
                config[s][k + "_Blue"]  = str(int(colorSchemes[i][k][2]*255.+0.5) )
                config[s][k + "_Alpha"] = str(int(colorSchemes[i][k][3]*255.+0.5) )
        
        if not "PNG_GENERATION" in config: config["PNG_GENERATION"] = {}
        config["PNG_GENERATION"]["size"] = str(png_size)
        config["PNG_GENERATION"]["line_width"] = str(png_line_width)
        config["PNG_GENERATION"]["line_color_red"] = str(png_line_color[0])
        config["PNG_GENERATION"]["line_color_green"] = str(png_line_color[1])
        config["PNG_GENERATION"]["line_color_blue"] = str(png_line_color[2])
        config["PNG_GENERATION"]["line_color_alpha"] = str(png_line_color[3])
        config["PNG_GENERATION"]["fill_color_red"] = str(png_fill_color[0])
        config["PNG_GENERATION"]["fill_color_green"] = str(png_fill_color[1])
        config["PNG_GENERATION"]["fill_color_blue"] = str(png_fill_color[2])
        config["PNG_GENERATION"]["fill_color_alpha"] = str(png_fill_color[3])

        f = open("apps/python/map_display/map_display.ini", "w")
        config.write(f)
        f.close()
        
    def render_callback(self):
        #acdebug("render_calback called")
        ac.glColor4f(1.0,1.0,1.0,1.0)
        if self.recording:
            x,y,z = ac.getCarState(0, acsys.CS.WorldPosition)
            p = [x,y,z]
            
            if len(self.recordedPoints) > 0:
                self.recordedPoints[-1]
                de = distance(p, self.recordedPoints[-1])
            else:
                de = self.delta_s+1.
            
            if len(self.recordedPoints) > 5:
                ds = distance(p, self.recordedPoints[0])
            else:
                ds = self.delta_s+1.
            
            if de >= self.delta_s and ds >= self.delta_s:
                self.recordedPoints.append(p)
            elif ds < self.delta_s:
                self.recordedPoints.append(p)
                self.recording = False
                self.setStatusBarText("Recording stopped.")
        self.render()
        self.check_generated_pngs()
        ac.glColor4f(1.0,1.0,1.0,1.0)
    
    def get_track_file(self):
        script_file = os.path.realpath(__file__)
        script_dir = os.path.dirname(script_file)
        track_file = os.path.join(script_dir, "data", self.md_data_prefix + ".pickled")
        track_file = os.path.normpath(os.path.relpath(track_file))
        return track_file
    
    def write_map(self):
        if len(self.recordedPoints) < 2:
            acinfo("no track to save")
            return
        track_file = self.get_track_file()
        tfs = os.path.splitext(track_file)
        k = 0
        # search for a non-existing file
        while os.path.exists(track_file):
            track_file = tfs[0] + ("-%d" % k) + tfs[1]
            k += 1
        f = open(track_file, "wb")
        pickle.dump(self.recordedPoints, f, 2)
        f.close()
        acinfo("written track file to %s" % track_file)
        script_file = os.path.realpath(__file__)
        script_dir = os.path.dirname(script_file)
        if distance(self.recordedPoints[0], self.recordedPoints[-1]) <= self.delta_s:
            fill_color = "0x%02x%02x%02x%02x" % (png_fill_color)
        else:
            fill_color = "0x00000000"
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        call = [script_dir + "/create_png.exe", "-s", str(png_size), "-p", track_file, str(png_line_width), "0x%02x%02x%02x%02x" % png_line_color, fill_color]
        p = subprocess.Popen(call, startupinfo=startupinfo)
        self.image_creation_processes[track_file] = p
        acinfo("external call %s" % str(call))
        return track_file
        
    def render(self):
        display = None
        recDisplay = None
        if len(self.maps) > 0:
            display = self.map_displays[self.maps[self.currentMapIdx]]
        if len(self.recordedPoints) > 1:
            if self.recDisplay is None:
                self.recDisplay = MapPointListDisplay(self, self.recordedPoints, [0.6, 0.1, 0.1])
            else:
                self.recDisplay.mergePoints(self.recordedPoints)
            recDisplay = self.recDisplay
            if display is None:
                display = recDisplay
                recDisplay = None
        else:
            self.recDisplay = None
        if not display is None:
            self.scale, self.offsetx, self.offsety = display.getTransformation()
            self.assertEgoVisible()
            display.render()
            if not recDisplay is None:
                recDisplay.setTransformation(self.scale, self.offsetx, self.offsety)
                recDisplay.render()
            self.render_cars()
            self.checkTimer()
        
    def assertEgoVisible(self):
        if self.zoom != 1.:
            x,y,z = ac.getCarState(0, acsys.CS.WorldPosition)
            dx = x*self.scale+self.offsetx
            dy = z*self.scale+self.offsety
            # search for coffsetx, coffsety that are centering the ego
            #   dx*self.zoom+coffsetx = (x_app_size-left_space-right_space)/2+left_space
            #   dy*self.zoom+coffsety = (y_app_size-bottom_space-top_space)/2+top_space
            self.coffsetx = (x_app_size-left_space-right_space)/2+left_space - dx*self.zoom
            self.coffsety = (y_app_size-bottom_space-top_space)/2+top_space  - dy*self.zoom
        else:
            self.coffsetx = 0.
            self.coffsety = 0.
            
    def zoomTransform(self, x, y):
        return (self.zoom*x + self.coffsetx, self.zoom*y + self.coffsety)
        
    def glVertex2f(self, x, y):
        return ac.glVertex2f(*self.zoomTransform(x, y))
    
    def glQuad(self, x, y, w, h):
        tx, ty = self.zoomTransform(x,y)
        tw = w*self.zoom
        th = h*self.zoom
        return ac.glQuad(tx, ty, tw, th)

    def glQuadTextured(self, x, y, w, h, tex):
        tx, ty = self.zoomTransform(x,y)
        tw = w*self.zoom
        th = h*self.zoom
        return ac.glQuadTextured(tx, ty, tw, th, tex)
                
    def getOpponentState(self, plyLapCount, plySplinePosition, plyOutLap, oppLapCount, oppSplinePosition, oppOutLap):
        session_type = workarounds.acGetSessionType()
        if session_type == "race":
            if plyLapCount == oppLapCount:
                if plySplinePosition >= oppSplinePosition:
                    return self.BEHIND_SAME_LAP
                else:
                    return self.AHEAD_SAME_LAP
            elif plyLapCount == oppLapCount+1:
                if plySplinePosition >= oppSplinePosition:
                    return self.BEHIND_OVERLAPPED
                else:
                    return self.BEHIND_SAME_LAP
            elif plyLapCount > oppLapCount+1:
                return self.BEHIND_OVERLAPPED
            elif plyLapCount == oppLapCount-1:
                if plySplinePosition >= oppSplinePosition:
                    return self.AHEAD_SAME_LAP
                else:
                    return self.AHEAD_OVERLAPPED
            else: #plyLapCount < oppLapCount-1
                return self.AHEAD_OVERLAPPED
        else:
            if plyOutLap and not oppOutLap:
                return self.AHEAD_OVERLAPPED
            elif not plyOutLap and oppOutLap:
                return self.BEHIND_OVERLAPPED
            elif plyOutLap and oppOutLap:
                return self.BEHIND_SAME_LAP
            else:
                return self.AHEAD_SAME_LAP
        return self.UNKNOWN_OPP_STATE
        
    def render_cars(self):
        specId = ac.getFocusedCar()
        plyLapCount = ac.getCarState(specId, acsys.CS.LapCount)
        plySplinePosition = ac.getCarState(specId, acsys.CS.NormalizedSplinePosition)
        numCars = ac.getCarsCount()
        allcars = list(range(numCars-1, -1, -1))
        for carid in allcars:
            if ac.isConnected(carid) and not carid in self.carWrappers:
                self.carWrappers[carid] = CarWrapper(carid)
            if not ac.isConnected(carid) and carid in self.carWrappers:
                del self.carWrappers[carid]
            if carid in self.carWrappers:
                if ac.isCarInPitline(carid):
                    self.carWrappers[carid].out_lap = True
                lc = ac.getCarState(carid, acsys.CS.LapCount)
                if self.carWrappers[carid].lastLapCount < lc:
                    self.carWrappers[carid].out_lap = False
                self.carWrappers[carid].lastLapCount = lc
        if specId in self.carWrappers:
            plyOutLap = self.carWrappers[specId].out_lap
        else:
            plyOutLap = False
        pxHeadingArrowFront = 8 * self.arrow_size
        pxHeadingArrowBack = 6 * self.arrow_size
        pxHeadingArrowSide = 6 * self.arrow_size
        for carid in allcars:
            n = ac.getDriverName(carid)
            x,y,z = ac.getCarState(carid, acsys.CS.WorldPosition)
            splPos = ac.getCarState(carid, acsys.CS.NormalizedSplinePosition)
            if carid in self.carWrappers:
                if carid == specId:
                    # this is the players car, always display green
                    color = colorSchemes[self.currentColorScheme]['colorEgo']
                else:
                    # this is an opponent.
                    # check the opponent state
                    #oppLapCount = workarounds.acGetLapCount(carid)
                    oppLapCount = ac.getCarState(carid, acsys.CS.LapCount)
                    oppSplinePosition = splPos
                    oppOutLap = self.carWrappers[carid].out_lap
                    oppState = self.getOpponentState(plyLapCount, plySplinePosition, plyOutLap, oppLapCount, oppSplinePosition, oppOutLap)
                    color = self.oppColors[oppState]
                    #acdebug("carid=%d, name=%s, splPos=%f lapCount=%d wx=%f wy=%f wz=%f" % (carid, n, oppSplinePosition, oppLapCount,x,y,z))
                dx = x*self.scale+self.offsetx
                dy = z*self.scale+self.offsety
                # TODO: would like to use heading vector, but not (yet?) available in ac API
                v1 = ac.getCarState(carid, acsys.CS.TyreHeadingVector, acsys.WHEELS.RL)
                if type(v1) == type(()):
                    v2 = ac.getCarState(carid, acsys.CS.TyreHeadingVector, acsys.WHEELS.RR)
                    vx = vx1+vx2
                    vy = vy1+vy2
                    vz = vz1+vz2
                else:
                    # error, probably this API is not yet implemented. Use velocity vector instead
                    vx,vy,vz = ac.getCarState(carid, acsys.CS.Velocity)
                ac.glColor4f(*color)
                v = math.sqrt(vx**2+vz**2)
                if v > 0.1:
                    vx /= v
                    vz /= v
                    nx = -vz
                    nz = vx
                    ac.glBegin(acsys.GL.Triangles)
                    self.glVertex2f(dx, dy)
                    self.glVertex2f(dx-vx*pxHeadingArrowBack+nx*pxHeadingArrowSide, 
                                    dy-vz*pxHeadingArrowBack+nz*pxHeadingArrowSide)
                    self.glVertex2f(dx+vx*pxHeadingArrowFront, 
                                    dy+vz*pxHeadingArrowFront)
                    ac.glEnd()
                    ac.glBegin(acsys.GL.Triangles)
                    self.glVertex2f(dx, dy)
                    self.glVertex2f(dx+vx*pxHeadingArrowFront, 
                                    dy+vz*pxHeadingArrowFront)
                    self.glVertex2f(dx-vx*pxHeadingArrowBack-nx*pxHeadingArrowSide, 
                                    dy-vz*pxHeadingArrowBack-nz*pxHeadingArrowSide)
                    ac.glEnd()
                else:
                    self.glQuad(dx-3*self.arrow_size, dy-3*self.arrow_size, 7*self.arrow_size, 7*self.arrow_size)
    
    def click_app_window(self):
        acdebug("click_app_window")
        self.setInteractive(True)
    
    def setInteractive(self, mode):
        acdebug("setInteractive")
        self.interactiveMode = mode
        for b in self.buttons:
            ac.setVisible(b, mode)
        if self.recording:
            # regardless of the timer, we show the recording button, if we are in record mode
            ac.setVisible(self.btnRec, True)
        if mode:
            self.showMenuTime = time.clock()
        else:
            self.setStatusBarText("")
    
    def checkTimer(self):
        if self.interactiveMode:
            if time.clock() - self.showMenuTime > 10.:
                self.setInteractive(False)
        
    def setZoom(self, newZoom):
        acdebug("setZoom")
        if newZoom < 1.:
            newZoom = 1.
        elif newZoom > 10.:
            newZoom = 10.
        self.zoom = newZoom
        
    def getZoom(self):
        acdebug("getZoom")
        return self.zoom
        
    def toggleMap(self):
        acdebug("toggleMapStart cmapIdx=%d maps=%s" % (
            self.currentMapIdx, str(self.maps)))
        self.currentMapIdx += 1
        if self.currentMapIdx >= len(self.maps):
            self.currentMapIdx -= max(1,len(self.maps))
        if self.currentMapIdx >= 0 and self.currentMapIdx < len(self.maps):
            m = self.maps[self.currentMapIdx]
            acinfo("map_display.py: current map: %s" % m)
            self.setStatusBarText("Use Map: %s" % m)
        else:
            self.setStatusBarText("Invalid map: %d" % self.currentMapIdx)
        acdebug("toggleMapEnd cmapIdx=%d maps=%s" % (
            self.currentMapIdx, str(self.maps)))
            
    def colorScheme(self):
        self.currentColorScheme = (self.currentColorScheme + 1) % len(colorSchemes)
        i = self.currentColorScheme
        self.oppColors = {
            self.BEHIND_SAME_LAP : colorSchemes[i]['colorOppBehindSameLap'],
            self.BEHIND_OVERLAPPED : colorSchemes[i]['colorOppBehindOverlapped'],
            self.AHEAD_SAME_LAP : colorSchemes[i]['colorOppAheadSameLap'],
            self.AHEAD_OVERLAPPED : colorSchemes[i]['colorOppAheadOverlapped'],
            self.UNKNOWN_OPP_STATE : colorSchemes[i]['colorOppUnknown'],
        }
        self.setStatusBarText("Selected color scheme %d" % i)
        
    def startRecording(self):
        msg = "Recording started"
        if len(self.recordedPoints) > 0:
            msg = "Restarted, old rec. discarded"
        self.recording = True
        self.recordedPoints = []
        self.setStatusBarText(msg)
        self.setInteractive(self.interactiveMode)
        
    def discardRecording(self):
        self.recording = False
        self.recordedPoints = []
        self.setStatusBarText("Recording discarded")
        self.setInteractive(self.interactiveMode)
        
    def saveRecording(self):
        if len(self.recordedPoints) >= 2:
            file = self.write_map()
            self.maps.append(file)
            self.map_displays[file] = MapPointListDisplay(self, file)
            acinfo("Appended point list to displays.")
            self.setStatusBarText("Saved: %s" % file)
            # use the new map for display
            self.currentMapIdx = self.maps.index(file)
        else:
            self.setStatusBarText("Nothing to save")
        self.recording = False
        self.recordedPoints = []
        self.setInteractive(self.interactiveMode)
        
    def check_generated_pngs(self):
        tracks = list(self.image_creation_processes.keys())
        for t in tracks:
            p = self.image_creation_processes[t]
            p.poll()
            if p.returncode == 0:
                acinfo("create_png finished successfully")
                png = os.path.splitext(t)[0] + ".png"
                display = MapPngDisplay(self, png, self.track_name, self.track_config)
                self.maps.append(png)
                self.map_displays[png] = display
                self.setStatusBarText("Converted to png file")
                self.currentMapIdx = self.maps.index(png)
            if not p.returncode is None:
                acinfo("create_png finished unsuccessfully (return code %d)" % p.returncode)
                del self.image_creation_processes[t]
                self.setInteractive(True)

    def setStatusBarText(self, text):
        if len(text) > 30:
            text = text[:10]+"..."+text[-17:]
        #ac.setText(self.statusBar, text)
        ac.setTitle(appWindow, text)
    
def acMain(ac_version):
    try:
        acinfo("Never Eat Yellow Snow APPS (map_display %s)" % map_display_lib.version)
        global appWindow
        appWindow = ac.newApp("MapDisplay")
        ac.setSize(appWindow, x_app_size, y_app_size)
        if use_transparent_hacks:
            # hide title bar hacks provided by snjper:
            ac.setTitle(appWindow, "")
            ac.drawBorder(appWindow, 0)
            ac.setBackgroundOpacity(appWindow, background_opacity)
            ac.setIconPosition(appWindow, 0, -9000)
        global mapDisplay
        mapDisplay = MapDisplay(appWindow)
        ac.addRenderCallback(appWindow, render_callback)
        ac.addOnClickedListener(appWindow, on_click_app_window)
        acinfo("map_display initialized successfully")
    except:
        acerror(traceback.format_exc())

def acShutdown():
    try:
        mapDisplay.close()
    except:
        acerror(traceback.format_exc())
        
def render_callback(deltaT):
    try:
        render_work()
    except:
        acerror(traceback.format_exc())

def render_work():
    try:
        if use_transparent_hacks:
            # hide title bar hacks provided by snjper:
            ac.setBackgroundOpacity(appWindow, background_opacity)
            ac.drawBorder(appWindow, 0)
        mapDisplay.render_callback()
    except:
        acerror(traceback.format_exc())
        
def on_click_app_window(arg1, arg2):
    try:
        acdebug("on_click_app_window (%s, %s)" % (str(arg1), str(arg2)))
        mapDisplay.click_app_window()
    except:
        acerror(traceback.format_exc())
    
def on_click_zoom_in(arg1, arg2):
    try:
        acdebug("on_click_zoom_in")
        mapDisplay.setZoom(mapDisplay.getZoom()*1.5)
    except:
        acerrro(traceback.format_exc())

def on_click_zoom_out(arg1, arg2):
    try:
        acdebug("on_click_zoom_out")
        mapDisplay.setZoom(mapDisplay.getZoom()/1.5)
    except:
        acerror(traceback.format_exc())

def on_click_zoom_def(arg1, arg2):
    try:
        acdebug("on_click_zoom_def")
        mapDisplay.setZoom(1.0)
    except:
        acerror(traceback.format_exc())

def on_click_change_map(arg1, arg2):
    try:
        acdebug("on_click_change_map")
        mapDisplay.toggleMap()
    except:
        acerror(traceback.format_exc())

def on_click_color_scheme(arg1, arg2):
    try:
        acdebug("on_click_color_scheme")
        mapDisplay.colorScheme()
    except:
        acerror(traceback.format_exc())

def on_click_start_rec(arg1, arg2):
    try:
        acdebug("on_click_start_rec")
        mapDisplay.startRecording()
    except:
        acerror(traceback.format_exc())

def on_click_discard_rec(arg1, arg2):
    try:
        acdebug("on_click_discard_rec")
        mapDisplay.discardRecording()
    except:
        acerror(traceback.format_exc())
        
def on_click_save_rec(arg1, arg2):
    try:
        acdebug("on_click_save_rec")
        mapDisplay.saveRecording()
    except:
        acerror(traceback.format_exc())
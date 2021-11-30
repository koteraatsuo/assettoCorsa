# sol_weather.py - by x4fab, leBluem, Blamer, demonized

bAddTransition                 = False # for smoother weather transitions when using plan
iTransitiontime                = 3.0   # seconds for transition
bRemoveAfterTransitionTime     = False # remove the old weather for transition from the plan

sPatchVersion = ''
sVerCode = ''
sAppError = ''
errorMSG = ''
sVer = '11.6'
# 11.6- weather controls also hidden now with base weather
#     - using our own siminfo lib now to avoid errors with others
#     - fixed default hotkey settings
# 11.5- reverted to 10.6 and gone from there
#     - weather controls are hidden with sol2
#     - check for weather implementation is now on by default
# 10.6- fixed initial reading values from "ctrl_config.lua"
# 10.5- fixed not applying new weather plan
# 10.4- fixed some saving issues other small changes
# 10.3- green CM weather text was visible with KeepWeatherPlanSettings on
# 10.2- fix ツ
# 10.1- fix
# 10.0- made check for changes of weather implementation optional, off by default
#       added buttons to adjust app height
#       changed icon/app title to be not visible by default
#       fixed bug: did not write to empty settings file
#       removed "Hide Title" option, no title bar at all now
#
# version history in cronological order from here:
#
# v0.99 - added time acceleration by demonized, v1.0 - format
# v1.1 added reading start multiplier from race.ini, added 5x buttons
# v1.2+3 silly error corrections, only work with offset, not real values...
# v1.4 added stop time button, added "Hide title option" in CM, performance fixes
# v1.5 + and - time buttons fixed with time accel, days/s and weeks/s behaviour fixed
# v1.51 safer way to read race.ini
# v1.6 added basic sol dynamic plan enabler and selector by demonized, night brightness adjustment by leBluem
# v1.61 fixed reading of night__brightness from sol_config.lua
# v1.62 fixed scaling, moved date to titlebar
# v1.7 added force weather selector for overriding current plan by demonized
# v1.72 cleaner look and better scaling of plan selector
# v1.73 forcing weather considers current conditions
# v1.8 added symbols, rearranged time/date
# v1.81 re-added demonized' condition reading
# v1.82 changed buttons again, added timetravel
# v1.9 moved stuff again, changed timetravel destination weather
# v1.91 fixed time unsharpnes, v1.92 fixed debug 29.999 mph, v1.93 bigger buttons,
# v1.94 bigger buttons fix, v1.95 day multiplier fix
# v1.96 cleaned up, added tracklight switch and range slider (working for lights with range>5, to skip fake glow), better error handling
# v1.97 fixed restoring weather plan, fixed light toggle, v1.99 small ui fixs
# v2 log fix, v2.1,2,3 homedir fixs
# v2.5 added "Buttons on top..." option, added stealth mode whith "Hide close icon and titlebar..."
# weatherplan window remebers positon now
# v2.6 added "reset weather" button
# time reset button "↩" now jumps back to time of session start (before only multiplier was reset)
# added optional extra height for weatherwindow, added update interval settings,
# added saving/loading "Tracklights range" - saved value will be set after patch has loaded original config - must be enabled first
# v2.7 buttons on top stealthmode fix
# v2.8 other icons, fixed scaling, fixed more positions
# v2.9 fixed force weather
# v3.0 added resize button, added remove weather icons option
# v3.1 added performance mode; made checkbox-scaling and symbols optional, off by default
# v3.2 made all symbols optional, off by default
# v3.3 fix
# v3.4 changing weather does not reset tracklights to 100% like in prev versions; resize btn fix
# v3.5 symbols could work for everybody now ;), fix for w/symbols on, chkboxes wont get selected
# 3.6 last fixes
# 3.7 added weather plan creator; reload settings with resize button from its settings in CM
# 3.8 added autmatic rain+wiper adjustment, can be disabled in CM/settings/apps; fixed days-multiplier again
# 3.9 added car headlights range slider, reads current value,
#  when changing this, it permanently changes it in "_CurrentCar_.ini";
#  also this will reset any changes made to tracklights with the app
#  added "add random" weather button; added buttons for track/car config files
# 4.0 fixed speed i hope, added loop option for plan-creation
# 4.1 finally fixed speed, limited to +-180days/s ;
#  "add random" weather now randomizes all parameters
#  added "Minimal" button + option, that shows only time + weather + upcoming weather
#  added "Enable WeatherFX" button, when you forgot to enable it ;)
# 4.2 fixed not working whith shaders patch off
# 4.3 fixed stuff, performance mode will saved too now
# car headlights sliders now works (still resets tracklights)
# rain/wiper definitions now at the beginning of "SolWeatherApp.py", respect comments in sol_config
# 4.4 fixed saving sol performance mode
# 4.5 fixed ^
# 4.6 removed ^
# 4.7 renamed to "sol_weather"
# 4.8 added own symbols+fixed ingame display-name to "Sol Weather"
# 4.9 small adjustment: removed rain with haze/fog/mist
# 5.0 less strict on ascii files
# 5.1 wipers get faster/slower in steps
# 5.2 added manual rain/wiper control; auto wipers speed bound to RainAmount now
# 5.3 fixed multiplier issue; fixed manual rain/wipers-stuff when using first time; added saving manual rain/wiper values; refined settings page in cm
# 5.4 fixed saving tracklights range
# 5.5 fixed saving tracklights range again + minor ui fixes
# 5.6 fixes, changed error handling, added close button to sol-plan-window, app window-height range now +-600
# 5.7 fixed borked easter egg
# 5.8 "autorain" now handled by sol itself, app only sets RainParams in "sol_config.lua"
# if new sol 1.2 variables are not there, it defaults to v5.7 behaviour
# changed easter egg using its own plan now, switches back to what was before + other minor fixes
# 5.9 switching weather now also does reset weather
# 6.0 correctly reading values at start; better feedback on what is selected; performance mode turns off rain too now
# 6.1 fixed switching weather
# 6.2 fixed switching weather again
# 6.3 make last plan enabled again, when deselecting a forced weather/perf. mode
# 6.4 disable plans on exit, shows correct online weather from logfile, w-app controls disabled with online
# 6.5 reset weather on ppFilter change
# 6.6 added option in CM to turn off "reset weather on ppFilter change", same to keep ingame selected weather plan
# fixed more stuff than i can think of
# 6.7 fixed ctype error
# 6.8 fixed saving weather plan window, b2f easter egg does more than changing weather
# 6.9 fixed enabling checkbox when changing plan, small mistake for using defaults
# 6.9a no logstuff in acupdate
# 7.0 fixed reading car file in acupdate
# 7.1 fixed wrongly setting dynamic weather at start; performance mode correctly turns off rain; fixed minor error in log; fixed some easteregg stuff
# 7.2 removed performance mode, since its the same as no clouds
# added glow slider, changes glow intensity for tracklights in memory, no changes to config itself
# added spinners for LIT_MULT, CAR_LIGHTS_LIT_MULT and SPECULAR_MULT per track
#  -note: values saved in [LIGHTING] section at end of session
#  -if you look at the track-config during game, there are still the original values
# added close button to wApp, addedd toggle BG button to weatherplan
# added glow to saved options if its enabled
# in CM options: added "MaxValues" and "Steps" for spinners, for headlights its doubled
# nearly fixed reset time button, but its a bit unsharp still :wink:
# fixed a bug when changing something, it took very long, now its smooth as silk
# 7.4 fixed transparency
# 7.5 fixed removing comments from config files
# 7.6 fixed bug, where sol_weather app is not visible, but plan window is
# 7.7 fixed more errors
# removed car-headlights-range-slider on demand
# added keyboard-shortcuts for time and to hide/show both windows
# CTRL+ALT+W    toggle main app
# CTRL+ALT+S    toggle plan window
# CTRL+ALT+E/D  time +/- 6 hours
# CTRL+ALT+R/F  time +/- 45 min
# CTRL+ALT+T/G  time +/- 5 min
# 7.8 fixed inconsistency btw close buttons and shortcuts
# 7.9 fixed wrong showing/saving SPECULAR_MULT
# 8.0 track config saved only when needed (was always saved with current timestamp after session) and fixed reading when there is no config at all
# 8.1 fixed loading shortcuts if changed in CM ui
# 8.2 parameterized time +/- buttons
# 8.3 fixed bug after a new track + session, app would still think its on previous track; shortcuts can be turned off now in CM-ui
# 8.4 fixed running tracks without config again
# 8.5 fixed tracklight range slider and edit track config button; added day of week to date
# 8.6 fixed display cfg name, when "ext_config.ini" is in "tracks\track\extension\"
# 8.7 fixed the rare case of per layout track-configs
# 8.8 per layout track-configs now correct
# 8.9 - car-headlights-range back, may not work for all cars! ('HEADLIGHTS_RANGE_MULT' from lighting_fx.ini)
#     - added slider for smoke-amount ('HEATING_EXTRA' from particlesFX.ini)
#     - added button to show/hide some options, "⛶" button on time-window now toggles transparency too, , like "⛶" in plan window
#     - now optional: tracklights range and glow, will be hidden in app-UI if not activated
#     --this can also be switched to "per track" mode
#     - added more possible modifiers for shortcuts
#     - some minor changes to ui and debug-output
#     - changed order and apperance of CM option page
# 9.0 - fixed when clicking car-cfg-file button, copying guessed config (if exists)
# 9.1 - fixed getting "ext_config.ini" from car folder
# 9.2 - fixed smoke-amount slider, added smoke-thickness slider and on/off for "Seasonal adjustments"
#       (sets both values in "weather_fx.ini" and "track_adjustments.ini")
# 9.3 - fixed creating files if not there in "documents\a...c...\cfg\extension"
# 9.4 - fixed reading "Seasonal adjustments" settings, active if set in whether "track_adjustments.ini" or "weather_fx.ini"
# 9.5 - when saving TracklightsRange/Glow enabled, dont save if still on default values (so track-configs doesn't change date and isn't altered)
# 9.6 - changed default key-modifier in CM options to "ALT+CTRL" (was ALT only before, which interfered with some ac-keys)
#     - with load/save plan settings on: fixed setting gui according to SOL__WEATHER_PLAN.lua on startup
#     - removed useless enable patch/sol buttons
#     - fixed nasty ini write bug, where it would write the first matching KEY but in the wrong section
#     - setting smoke values sets smoke ON if not already, both=0 -> cspSmoke=Off
# 9.7 - fixed first using shortcut to hide app windows, when it is still on from last session
#     - changed used dummy filename from "extension/weather/sol/reset_dummy.txt" to ".../reset_dummy.lua"
#     - small fix when saving weather settings is on; got rid of index out of range error in log
#     - changed default hotkey for toggling plan-window from CTRL+ALT+Q to CTRL+ALT+S,
#     - as this is the default hotkey for damage displayer
# 9.8 - fixed nasty flicker bug with rain, also improved performance slightly
#     - cleaned up ui and added some buttons
#     - updated to work on linux too, thx kl0n3er, but not 100% yet
#     - made the app react to changes in weather implementation and controller
#     - some ui display fixes like weather display with
#       enabled "sol debug weather change" in sol_config app
# 9.9 - mostly a workaround for rain situation
# 9.91- added app registering track config in layout folder
# 9.92- fixed bug where solw-app writes in track-config at wrong place and effectivly breaking this config
#     - atm unfortunately changing glow with Save/Load tracklights on is not working
# 9.93- fixed some debug text issues; no loop for debugtext used anymore
# 9.94- fixed app switching text with vanilla ac
# 9.95- both smoke settings now in steps of 50 and with lower max
# 9.96- added second button for secondary track-config if exist, also applying tracklightrange and -glow working again
# 9.97- fixed track-config button
#     - some minor ui-adjustments
#     - removed hide weather option in CM ui
# 9.98- carefully added different fonts to choose from for ui

# defaults changed later by reading config
sCustomFontName = "default"
bUseCustomFont = False
gEnableHotKeys = True
gModifier = 'ALT+CTRL'
hotKeyToggleMain = "W"
hotKeyTogglePlan = "S"
hotKeyPlusBig = "E"
hotKeyMinusBig = "D"
hotKeyPlusSmall = "R"
hotKeyMinusSmall = "F"
hotKeyPlusTiny = "T"
hotKeyMinusTiny = "G"
hotKeyListener = 0

# note: edit this file with an utf_8/UNICODE editor ONLY, you have been warned!
# ˂˃˄˅            ⋁⋀⌵ ⌄⌃
# ◃▹◅▻△▽◁▷ ◂▸◄►◀▶ <> ≪≫ ⫷⫸ ⋘⋙
# ▵▿∆∇△▽ ▴▾⏶⏷▲▼  ⏴⏵⏮⏭ ⇀ ⇉ ⇒ ⇛ ⇝ ⇢ ⇥ ⇨ ↞↠↢↢↢↣ → ⤳ ↜↭↝ ↦ ⅏ ⏫⏬⏩⏪⏩⏪ ⏸ ▩▨▧▦▥▤ ░ ▒ ▓
# ⤊⤋ ⇑⇓ ⇧⇩⇪ ↑↓ ⇈ ⇊ ⇣⇡ ↟↡ ⩓⩔⋰⋱⋗⋖≻≺ ⚟⚞⟀⬱⇶⇚⇛
# ⛐✈✇✆✅✄✃✂✁✀⛿⛾⛽⛼⛻⛺⛹⛸⛷❌❍❎❏❐❑❒⧘⧙⧚⧛⧗⧖⧕⧔⧓⧑⧒⧏⧐⧎ ∙ ⁚ ⁝ ┋ ┊ |
# ⛱ ✨☀☼☄☁⛅☂☔⛆⛈⌁☇☈↯⛄☃⛇ ⃰ ⁂ ⁑⁕   ☾ ☽ ⌚⏰⏱⏲⏳  ↩       ․ ‥ …   ┉ ┈  ⃛  ⃜  ⁗ ′ ″ ‴ ‶ ‷ ⁗
# ⚛ ☢ ☣ ⋆ ★ ☆ ⚹ ✱ ✲ ✳ ✴ ✵ ✶ ✷ ✸ ✹ ✺ ✻ ✼ ✽ ✾ ✿ ❀ # ➿ ∞ ⎈ ☼ ⁜ ⚵ ⚶ ⚷ ⚸ ∾ ∻≁≈ ∿≋ ≌ ≎ ∼∽ ⊸ ⁕※

weathersNames = (
    "✺ NoClouds", "✹ Clear", "∿ FewClouds", "∻ ScatteredClouds", "✾ Windy", "≁ BrokenClouds", "∾ OvercastClouds",
    "▒ Mist", "▓ Fog", "░ Haze", "⁚ Dust", "⁝ Sand", "⅏ Smoke",
    "☂ LightDrizzle", "☂ Drizzle", "☂ HeavyDrizzle", "☔ LightRain", "☔ Rain", "☔ HeavyRain",
    "⌁ LightThunderstorm", "☇ Thunderstorm", "↯ HeavyThunderstorm", "↠ Squalls", "↭ Tornado", "⇛ Hurricane",
    "⁕ LightSnow", "⁑ Snow", "⁂ HeavySnow", "⚹ LightSleet", "⚹ Sleet", "⚹ HeavySleet", "✵ Hail")
weathers = (
    "NoClouds", "Clear", "FewClouds", "ScatteredClouds", "Windy", "BrokenClouds", "OvercastClouds",
    "Mist", "Fog", "Haze", "Dust", "Sand", "Smoke",
    "LightDrizzle", "Drizzle", "HeavyDrizzle", "LightRain", "Rain", "HeavyRain",
    "LightThunderstorm", "Thunderstorm", "HeavyThunderstorm", "Squalls", "Tornado", "Hurricane",
    "LightSnow", "Snow", "HeavySnow", "LightSleet", "Sleet", "HeavySleet", "Hail")
sNewWeather = "new plan's filename, first 2 clicks adds weather, .lua added autom.\n\n\nParameters for next weather added from the left:\n  -time holding in minutes\n  -time changing in minutes\n  -ambient Temperature in °C\n  -road temperature in °C\n  -wind direction in degrees°\n  -wind speed in km/h"
sNewWeatherRandom = "\n\n\n\nrandom +-30min\nrandom +-10min\nrandom +-5°C\nrandom +-5°C\nrandom +-45°\nrandom +-20km/h"
sNewWeatherClick = 'Click on a weather to add it here!\nWhen done use "Save new plan"\nClick "<" again to restart with a new plan'
# from 0..1, in percent/100; not used in sol>1.2
sRainAmounts = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # NoClouds, Clear, FewClouds, ScatteredClouds, Windy, BrokenClouds, OvercastClouds,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Mist, Fog, Haze, Dust, Sand, Smoke,
                0.2, 0.3, 0.4, 0.5, 0.6, 0.75, # LightDrizzle, Drizzle, HeavyDrizzle, LightRain, Rain, HeavyRain,
                0.6, 0.8, 1.0, 0.0, 0.5, 1.0, # LightThunderstorm, Thunderstorm, HeavyThunderstorm, Squalls, Tornado, Hurricane,
                0.1, 0.2, 0.3, 0.025, 0.1, 0.2, 0.75) # LightSnow, Snow, HeavySnow, LightSleet, Sleet, HeavySleet, Hail)
# used for setting temperature for all the different weathers
# road temp will always be 5°C more
sVarTemps    = (40.0, 35.0, 30.0, 25.0, 20.0, 15.0, 12.5, # NoClouds, Clear, FewClouds, ScatteredClouds, Windy, BrokenClouds, OvercastClouds,
                8.0, 4.0, 12.0, 17.0, 23.0, 27.0, # Mist, Fog, Haze, Dust, Sand, Smoke,
                15.0, 12.5, 10.0, 13.5, 11.5, 6.0, # LightDrizzle, Drizzle, HeavyDrizzle, LightRain, Rain, HeavyRain,
                20.0, 18.0, 16.0, 14.0, 12.0, 10.0, # LightThunderstorm, Thunderstorm, HeavyThunderstorm, Squalls, Tornado, Hurricane,
                -1.0, -5.0, -10.0, -2.5, -12.5, -15.0, 6.5) # LightSnow, Snow, HeavySnow, LightSleet, Sleet, HeavySleet, Hail)
# used for setting windspeed for all the different weathers
sVarWind     = (0.0, 2.5, 5.0, 10.0, 40.0, 20.0, 25.0, # NoClouds, Clear, FewClouds, ScatteredClouds, Windy, BrokenClouds, OvercastClouds,
                4.0, 2.0, 0.0, 5.0, 10.0, 15.0, # Mist, Fog, Haze, Dust, Sand, Smoke,
                20.0, 25.0, 30.0, 40.0, 50.0, 60.0, # LightDrizzle, Drizzle, HeavyDrizzle, LightRain, Rain, HeavyRain,
                70.0, 80.0, 100.0, 120.0, 150.0, 200.0, # LightThunderstorm, Thunderstorm, HeavyThunderstorm, Squalls, Tornado, Hurricane,
                5.0, 10.0, 20.0, 15.0, 25.0, 35.0, 40.0) # LightSnow, Snow, HeavySnow, LightSleet, Sleet, HeavySleet, Hail)

import ac, acsys
import sys, os, platform, traceback, threading
import random, time, datetime, calendar
import io, string, configparser, re
import codecs
from shutil import copyfile

# from stat import *
if platform.architecture()[0] == "64bit":
    dllfolder = "stdlib64"
else:
    dllfolder = "stdlib"
cwd = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(cwd, dllfolder))
os.environ['PATH'] = os.environ['PATH'] + ";."
import ctypes
from ctypes import wintypes
from extlibs import win32con # , win32api

userDir=''
CSIDL_PERSONAL = 5 # My Documents
SHGFP_TYPE_CURRENT = 0 # Get current, not default value
buf = ctypes.create_unicode_buffer(ctypes.wintypes.MAX_PATH)
ctypes.windll.shell32.SHGetFolderPathW(None, CSIDL_PERSONAL, None, SHGFP_TYPE_CURRENT, buf)
userDir = buf.value # bingo, userdir/documents!

from acinfolib.acsiminfo import acinfo

x=0
try:
    i=acinfo.physics.localVelocity
    x+=1
    i=acinfo.graphics.windDirection
    x+=2
    i=acinfo.static.pitWindowEnd
    x+=4
except:
    ac.log("")
    ac.log("SolWeatherApp: WARNING sim_info.py conflict (index "+str(x)+") with older version loaded by previous app")
    ac.log("SolWeatherApp: This could cause SolWeatherApp to fail, update other apps 'sim_info.py' or untick previous loaded apps")
    ac.console("SolWeatherApp: WARNING sim_info.py conflict (index "+str(x)+") with older version loaded by previous app")
    ac.console("SolWeatherApp: This could cause SolWeatherApp to fail, update other apps 'sim_info.py' or untick previous loaded apps")
    ac.log("")


raceinipath      = userDir + "/Assetto Corsa/cfg/race.ini"
racelogpath      = userDir + "/Assetto Corsa/logs/log.txt"
INIvideo         = userDir + "/Assetto Corsa/cfg/video.ini"
settingsFilePath = "apps/python/sol_weather/settings/settings.ini"
ac.log("SolWeatherApp reading config ... " + raceinipath)

documentsAppSettings    = "Assetto Corsa/cfg/apps/sol_weatherapp_settings.ini"
documentsSolTrackConfig = "Assetto Corsa/cfg/apps/sol_weatherapp_tracklightadjustments.ini"

# "%userprofile%/Documents/Assetto Corsa/cfg/extension/weather_fx.ini"
documentSectionGeneral = 'GENERAL'
ps_path                 = 'extension/weather-controllers/sol/'
sol_config              = 'extension/weather/sol/sol__config.lua'
sol_controllercfg       = 'extension/weather-controllers/sol/ctrl_config.lua'
# sol_dummy               = 'extension/weather/sol/reset_dummy.txt'
sol_dummy               = 'extension/weather/sol/reset_dummy.lua'
carpath2                = 'extension/config/cars/'
carpath1                = userDir + '/Assetto Corsa/cfg/extension/config/cars/'
sPatchGeneralini        = userDir + '/Assetto Corsa/cfg/extension/general.ini'
sPatchTrackAdjust       = userDir + '/Assetto Corsa/cfg/extension/track_adjustments.ini'
sPatchTrackAdjustDef    = os.path.abspath('extension/config/track_adjustments.ini')
sPatchConfigLights      = userDir + '/Assetto Corsa/cfg/extension/lighting_fx.ini'
sPatchConfigLightsDef   = os.path.abspath('extension/config/lighting_fx.ini')
sPatchConfigParticles   = userDir + '/Assetto Corsa/cfg/extension/particles_fx.ini'
sPatchConfigParticlesDef= os.path.abspath('extension/config/particles_fx.ini')
sPatchWeatherFXini      = userDir + '/Assetto Corsa/cfg/extension/weather_fx.ini'
sPatchWeatherFXiniDef   = os.path.abspath('extension/config/weather_fx.ini')
plans_path              = ps_path + 'weather_plans/'
sol_weather_plan        = ps_path + 'SOL__WEATHER_PLAN.lua'

lastwfxINIdate = 0

bNewPlan                = False
onlinesession           = False
bForcedWeather          = False
bDynamicWeather         = False
gKeepPlanSettings       = False
gResetWeatherOnPPFilter = True
firstrun                = True
bBase                   = False
bSol2                   = False
bRainSettingsPresent    = False

lastfiledata = ''
last_plan    = ''
last_weather = ''
weather_orig = ''
ppFilterCurr = ''
ppFilterLast = ''

gAppsize = 1.0
gAppShowOnStart = False
gHideBG = False
gUpdateInterval = 0.25
gUseScaling = False
gMinimal = False  ### only show time of day + weather ; "True" or "False"
gWrapDebugText = False
gEast = False
gButtonH = 35
gButtonsAboveDateTime = False
gExtraHeight = 0
night__brightness_adjust = 0.5
gTracklightsRange = 100
gTracklightsGLOW = 100
gLoadSaveTracklightsRange = False
gLoadSaveTracklightsPerTrack = False
gUseSymbols = False
gShowSettings = False
gLightsON = True
gOrigHeadLightRange = -1.0
gOrigSmokeAmount = -1.0
gOrigSmokeThickness = -1.0
gAutoRain = True
gAutoWipers = True
gManuRain = False
gManuWipers = False
bPause = False
bwFXactive = False
gPlanLoop = False
gRandomTH = False
gRandomTC = False
gRandomAT = False
gRandomRT = False
gRandomWD = False
gRandomWS = False
gMaximumValues = 300
gSteps = 5

yyyyOrig=1985
mmOrig=10
ddOrig=26
hhOrig=1
mnOrig=10
yyyyWanted=1985
mmWanted=10
ddWanted=26
hhWanted=1
mnWanted=10
yyyyL=0
mmL=0
ddL=0
hhL=0
mnL=0
currsdate=''
currstime=''
timer = 0.0
timer2 = 0.0
timer3 = 0.0
timer4 = 0.0
speed = 0
lastspeed = 0
day_offset = 0
accel = 0
accel_offset = 0
accel_orig = 1
thread = 0
thread2 = 0
bVisibleMain = False
bVisiblePlan = False
bTimeCount = 0
bPatchActive=False
bTimeDone = True
bdoWIcheck = False
bTimeS = ""
sCar = ""
sTrack = ""
sLayout = ""
sFileTrack = ""
sFileTrack2 = ""
sFileCar = ""
sFileCar2 = ""
sFileTrackShort = ""
sFileCarShort = ""
sdatetime = ""
t=''
sdatetime=''
sdate = ''
stime = ''
smultacel=''
sWeatherName=''
sWeatherNameUpcoming=''
sRainWiper=''

spinnerRainV = 0
spinnerWipersV = 0
spinnerSoundV = 0
dt_obj = datetime.datetime(2019,1,1)
lastday = datetime.datetime(2019,1,1)
gOffsetBig     = int(60 * 60 * 6) # 6 hours
gOffsetSmall   = int(60 * 45)     # 45 mins
gOffsetSmaller = int(60 * 5)      # 5 mins

#app and controls
app = 0
lbTime = 0
lbDate = 0
lbMult = 0
lbChng = 0
lbWeat = 0
lbRang = 0
lbCar  = 0
lbDesc0 = 0
lbDesc1 = 0 # weather list left bc no scaling for chkboxes
lbDesc2 = 0 # weather list right
lbWeatherinfo = 0 # weather from CM
#buttons
timeOffsetMinusSmallerButton = 0
timeOffsetPlusSmallerButton = 0
timeOffsetMinusSmallButton = 0
timeOffsetPlusSmallButton = 0
timeOffsetMinusBigButton = 0
timeOffsetPlusBigButton = 0
timeOffsetMinusHugeButton = 0
timeOffsetPlusHugeButton = 0
timeOffsetMinusPlayButton = 0
timeOffsetPlusPlayButton = 0
timeOffsetMinusPlaySmallButton = 0
timeOffsetPlusPlaySmallButton = 0
timeOffsetMinusMonthButton = 0
timeOffsetPlusMonthButton = 0
timeOffsetMinusYearButton = 0
timeOffsetPlusYearButton = 0
timeAccelUpButton = 0
timeAccelDownButton = 0
timeAccelUpFasterButton = 0
timeAccelDownFasterButton = 0
timeAccelResetButton = 0
timeAccelStopButton = 0

btnShowSolSelector = 0
buttonResetWeather = 0
btnAppSizeUp = 0
btnAppSizeDn = 0
btnAppSizeHigher = 0
btnAppSizeLower = 0
btnTranspToggleAPP = 0
btnMinimalToggle = 0
btnSettingsToggle = 0

#plan selector
plans = []
ps_app = 0
edtFilename = 0
btnNewPlan = 0
btnNewPlansave = 0
btnPlanClose = 0
btnToggleTransparencyPS = 0
btnAppClose = 0
btnFileTrack = 0
btnFileTrack2 = 0
btnFileCar = 0
btnFileCar2 = 0
btnNewRandWeather = 0
lb_spinner_ps_plan = 0
lbNewPlan = 0 # button without onClick-proc, acting as text store
lbNewPlanWeatherDuration = 0 # also used for "new plan's filename"
lbNewPlanRandom = 0 # also used for "new plan's filename"
lbGlow = 0
spinner_ps_plan = 0
checkbox_ps_plan_enable = 0
checkbox_ps_weather = {}
checkbox_AutoRain = 0
checkbox_AutoWipers = 0
checkbox_ManualRain = 0
checkbox_ManualWipers = 0
checkbox_Tracklights = 0
chkBoxPlanLoop = 0
chkBoxRandomTH = 0
chkBoxRandomTC = 0
chkBoxRandomAT = 0
chkBoxRandomRT = 0
chkBoxRandomWD = 0
chkBoxRandomWS = 0
spinnerGlow = 0
spinnerRangeTrackLights = 0
spinnerRangeHeadLights = 0
spinnerSmokeAmount = 0
spinnerSmokeThickness = 0
spinnerTimeHolding = 0
spinnerTimeChanging = 0
spinnerTemperature_ambient = 0
spinnerTemperature_road = 0
spinnerWind_direction = 0
spinnerWind_speed = 0
spinnerRain = 0
spinnerWipers = 0
spinnerSound = 0

gLIT_MULT = 1.0
gCAR_LIGHTS_LIT_MULT = 1.0
gSPECULAR_MULT = 1.0
gSeasonsActive = 0

lbNigh = 0
lbLIT_MULT = 0
lbCAR_LIGHTS_LIT_MULT = 0
lbSPECULAR_MULT = 0
spinner_LIT_MULT = 0
spinner_CAR_LIGHTS_LIT_MULT = 0
spinner_SPECULAR_MULT = 0
spinnerNightBright = 0
checkbox_Seasons = 0

def ps_newrndweather(*args):
    global checkbox_ps_weather, lbChng, weathersNames, chkBoxRandomTH, chkBoxRandomTC, chkBoxRandomAT, chkBoxRandomRT, chkBoxRandomWD, chkBoxRandomWS
    global gRandomTH, gRandomTC, gRandomAT, gRandomRT, gRandomWD, gRandomWS
    global spinnerTimeHolding, spinnerTimeChanging, spinnerTemperature_ambient, spinnerTemperature_road, spinnerWind_direction, spinnerWind_speed
    global sVarTemps, sVarWind
    try:
        rndw = weathers[random.randint(0,31)]
        for k, v in checkbox_ps_weather.items():
            if k.encode('ascii', 'ignore').decode('ascii').strip() == rndw.encode('ascii', 'ignore').decode('ascii').strip():
                ac.setValue(v, 1)
            else:
                ac.setValue(v, 0)

        gRandomTH = bool(random.getrandbits(1))
        ac.setValue( chkBoxRandomTH, gRandomTH )
        gRandomTC = bool(random.getrandbits(1))
        ac.setValue( chkBoxRandomTC, gRandomTC )
        gRandomAT = bool(random.getrandbits(1))
        ac.setValue( chkBoxRandomAT, gRandomAT )
        gRandomRT = bool(random.getrandbits(1))
        ac.setValue( chkBoxRandomRT, gRandomRT )
        gRandomWD = bool(random.getrandbits(1))
        ac.setValue( chkBoxRandomWD, gRandomWD )
        gRandomWS = bool(random.getrandbits(1))
        ac.setValue( chkBoxRandomWS, gRandomWS )

        v=ac.getValue(spinnerTimeHolding)
        ac.setValue(spinnerTimeHolding, random.randint( int(max(1,v-v/2)), int(max(2,v+v/2)) ))
        v=ac.getValue(spinnerTimeChanging)
        ac.setValue(spinnerTimeChanging,random.randint( int(max(1,v-v/2)), int(max(2,v+v/2)) ))

        # v=ac.getValue(spinnerTemperature_ambient)
        v = sVarTemps[ weathers.index(rndw) ]
        ac.setValue(spinnerTemperature_ambient, random.randint(int(v-5), int(v+5) ))

        #v=ac.getValue(spinnerTemperature_road)
        v = sVarTemps[ weathers.index(rndw) ]+5.0
        ac.setValue(spinnerTemperature_road,    random.randint( int(v-5), int(v+5) ))

        #v=ac.getValue(spinnerWind_direction)
        ac.setValue(spinnerWind_direction, random.randint( int(0), int(360) ))

        #v=ac.getValue(spinnerWind_speed)
        v = sVarWind[ weathers.index(rndw) ]
        ac.setValue(spinnerWind_speed,     random.randint( int(max(0,v-8)), int(max(6,v+8)) ))

        ps_force_weather(rndw, 1)
    except:
        ac.setText(lbChng, '\n\n\n\n\n\n\n\n\n\n' + traceback.format_exc())

def chkboxchangedPlanLoop(name,value):
    global gPlanLoop
    gPlanLoop=False
    if value==True:
        gPlanLoop=True
def chkboxchangedTH(name,value):
    global gRandomTH
    gRandomTH=False
    if value==True:
        gRandomTH=True
def chkboxchangedTC(name,value):
    global gRandomTC
    gRandomTC=False
    if value==True:
        gRandomTC=True
def chkboxchangedAT(name,value):
    global gRandomAT
    gRandomAT=False
    if value==True:
        gRandomAT=True
def chkboxchangedRT(name,value):
    global gRandomRT
    gRandomRT=False
    if value==True:
        gRandomRT=True
def chkboxchangedWD(name,value):
    global gRandomWD
    gRandomWD=False
    if value==True:
        gRandomWD=True
def chkboxchangedWS(name,value):
    global gRandomWS
    gRandomWS=False
    if value==True:
        gRandomWS=True
def onSpinnerTime(*args):
    global lbNewPlanRandom, spinnerTimeHolding, spinnerTimeChanging, spinnerWind_speed
    i1 = ac.getValue(spinnerTimeHolding)/2
    i2 = ac.getValue(spinnerTimeChanging)/2
    i3 = ac.getValue(spinnerWind_speed)/2
    sNewWeatherRandom = "\n\n\n\nrandom +-" + str(i1) + "min\nrandom +-" + str(i2) + "min\nrandom +-5°C\nrandom +-5°C\nrandom +-45°\nrandom +-" + str(i3) + "km/h"
    ac.setText(lbNewPlanRandom, sNewWeatherRandom)

def runEditorTrack(*args): # btnFileTrack
    global sFileTrack, sTrack, sLayout
    if not os.path.isfile(sFileTrack):
        sFileTrack = 'extension/config/tracks/' + sTrack+'.ini'
        #if sLayout!='':
        #    sFileTrack = 'extension/config/tracks/' + sTrack+'_'+sLayout+'.ini'
        if os.path.isfile('apps/python/sol_weather/template_trackconfig.txt'):
            copyfile('apps/python/sol_weather/template_trackconfig.txt', sFileTrack)
        else:
            with codecs.open(sFileTrack, 'w', 'utf_8', errors='ignore') as file:
                file.write('; created by sol_weather app for track ' + sTrack + ' (' + sLayout + ')')
    runEditor(sFileTrack)

def runEditorTrack2(*args): # btnFileTrack2
    global sFileTrack2, sTrack, sLayout
    if os.path.isfile(sFileTrack2):
        runEditor(sFileTrack2)

def runEditorCar2(*args): # btnFileCar2
    global sFileCar2
    if os.path.isfile(sFileCar2):
        runEditor(sFileCar2)

def runEditorCar(*args): # btnFileCar
    global sFileCar, sCar, userDir
    try:
        if not os.path.isfile( sFileCar ):
            sFileCar = 'extension/config/cars/mods/' + sCar+'.ini'
            if os.path.isfile( userDir + '/Assetto Corsa/cfg/extension_cars_guessed/' + sCar + '.ini'):
                copyfile(      userDir + '/Assetto Corsa/cfg/extension_cars_guessed/' + sCar + '.ini', sFileCar )
                s=''
                with codecs.open(sFileCar, 'r', 'utf_8', errors='ignore') as file:
                    s = file.read()
                with codecs.open(sFileCar, 'w', 'utf_8', errors='ignore') as file:
                    file.write('; created by CustomShadersPatch'
                        + '; copied by sol_weather app from "/Assetto Corsa/cfg/extension_cars_guessed/"'
                        + '\n' + s)
            else:
                if os.path.isfile('apps/python/sol_weather/template_trackconfig.txt'):
                    copyfile('apps/python/sol_weather/template_trackconfig.txt', sFileTrack)
                else:
                    with codecs.open(sFileCar, 'w', 'utf_8', errors='ignore') as file:
                        file.write('# created by sol_weather app\n\n')
    except:
        ac.log('SolWeatherApp error: ' + traceback.format_exc())
    runEditor(sFileCar)

##############################################################################

def ps_AutoWipers(name,value):
    global checkbox_AutoWipers, checkbox_ManualWipers, gAutoWipers, gManuWipers
    global bPause
    if value==1:
        localpause=bPause
        bPause=True
        ac.setValue(checkbox_ManualWipers,0)
        bPause=localpause
        ac.setText(checkbox_AutoWipers, 'auto wipers')
    else:
        if gManuWipers:
            ac.setText(checkbox_AutoWipers, 'manual wipers')
        else:
            ac.setText(checkbox_AutoWipers, 'wiper ctrl off')
    if bPause==True:
        return
    gAutoWipers = False
    if value==1:
        gAutoWipers=True
        gManuWipers=False
        bPause=True
        ac.setValue(checkbox_ManualWipers,0)
        bPause=False

def ps_ManualWipers(name,value):
    global checkbox_AutoWipers, checkbox_ManualWipers, gAutoWipers, gManuWipers
    global bPause
    if bPause==True:
        return
    gManuWipers = False
    if value==1:
        gManuWipers=True
        gAutoWipers=False
        bPause=True
        ac.setValue(checkbox_AutoWipers,0)
        bPause=False
        ac.setText(checkbox_AutoWipers, 'manual wipers')
    else:
        if gAutoWipers:
            ac.setText(checkbox_AutoWipers, 'auto wipers')
        else:
            ac.setText(checkbox_AutoWipers, 'wiper ctrl off')

def ps_wipers(*args):
    global timer3, checkbox_AutoWipers, checkbox_ManualWipers, gManuWipers, gAutoWipers
    global bPause
    if bPause==True:
        return
    bPause=True
    ac.setValue(checkbox_AutoWipers,0)
    ac.setValue(checkbox_ManualWipers,1)
    ac.setText(checkbox_AutoWipers, 'manual wipers')
    bPause=False
    gManuWipers=True
    gAutoWipers=False
    timer3 = 10.0 # force update, dont wait 2 seconds stuff goes in HandleRain()

##############################################################################

def ps_AutoRain(name,value):
    global checkbox_AutoRain, checkbox_ManualRain, gAutoRain, gManuRain, sol_config, spinnerRain, sol_controllercfg
    global bPause
    global bRainSettingsPresent
    if bPause:
        return
    bPause = True
    if value==1:
        gAutoRain=True
        gManuRain=False
        ac.setText(checkbox_AutoRain, 'auto rain')
        if sol_controllercfg!='':
            appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_automatically', 'true')
        if bRainSettingsPresent:
            appWriteCFGValue(sol_config, '', 'weather__set_rain_automatically', 'true')
        ac.setValue(checkbox_ManualRain,0)
    else:
        gAutoRain = False
        if gManuRain:
            ac.setText(checkbox_AutoRain, 'manual rain')
            ac.setValue(checkbox_ManualRain,1)
        else:
            ac.setText(checkbox_AutoRain, 'rain ctrl off')
        ac.ext_pauseFsWatching()
        if sol_controllercfg!='':
            appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_automatically', 'false')
            appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_amount', '-1.0')
        if bRainSettingsPresent:
            appWriteCFGValue(sol_config, '', 'weather__set_rain_automatically', 'false')
            appWriteCFGValue(sol_config, '', 'weather__set_rain_amount', '-1.0')

        ac.ext_resumeFsWatching()
        appApplyTrackConfig()
    bPause = False

def ps_ManualRain(name,value):
    global checkbox_AutoRain, checkbox_ManualRain, sol_config, gAutoRain, gManuRain, spinnerRain, sol_controllercfg
    global bPause
    global bRainSettingsPresent
    if bPause==True:
        return
    bPause=True
    gManuRain = False
    if value==1:
        gManuRain=True
        gAutoRain=False
        ac.setValue(checkbox_AutoRain,0)
        bPause=False
        ac.setText(checkbox_AutoRain, 'manual rain')
        rainamount = float(ac.getValue(spinnerRain)) / 100.0
        ac.ext_pauseFsWatching()
        if sol_controllercfg!='':
            appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_amount', str(rainamount))
            appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_automatically', 'false')
        if bRainSettingsPresent:
            appWriteCFGValue(sol_config, '', 'weather__set_rain_amount', str(rainamount))
            appWriteCFGValue(sol_config, '', 'weather__set_rain_automatically', 'false')
        ac.ext_resumeFsWatching()
        appApplyTrackConfig()
    else:
        ac.setText(checkbox_AutoRain, 'rain ctrl off')
        ac.ext_pauseFsWatching() ##########
        if sol_controllercfg!='':
            appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_amount', '0.0')
            appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_automatically', 'false')
        if bRainSettingsPresent:
            appWriteCFGValue(sol_config, '', 'weather__set_rain_amount', '0.0')
            appWriteCFGValue(sol_config, '', 'weather__set_rain_automatically', 'false')
        ac.ext_resumeFsWatching() ##########
        appApplyTrackConfig()
    bPause=False

def ps_rain(*args):
    global checkbox_AutoRain, checkbox_ManualRain, sol_config, spinnerRain, gManuRain, gAutoRain, bPatchActive, sol_controllercfg
    global timer3, bPause, bBase
    global bRainSettingsPresent
    if bPause==True:
        return
    try:
        bPause=True
        ac.setText(checkbox_AutoRain, 'manual rain')
        ac.setValue(checkbox_ManualRain, 1)
        ac.setValue(checkbox_AutoRain, 0)
        gAutoRain=False
        gManuRain=True
        if bPatchActive:
            value=float( int(ac.getValue(spinnerRain)) ) / 100.0
            ac.ext_pauseFsWatching() #######
            if sol_controllercfg!='':
                appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_automatically', 'false')
                appWriteCFGValue(sol_controllercfg, '', 'weather__set_rain_amount', str(value))
            if bRainSettingsPresent:
                appWriteCFGValue(sol_config, '', 'weather__set_rain_automatically', 'false')
                appWriteCFGValue(sol_config, '', 'weather__set_rain_amount', str(value))

            #if bBase:
            RainParams = ac.ext_rainParams() # get current rain/wipers state from patch
            ac.ext_rainParamsAdjust( value, RainParams[1])
            #    # ac.ext_rainParamsSet(ac.getValue(spinnerRain)/100.0, RainParams[1])

            ac.ext_resumeFsWatching() #######
            appApplyTrackConfig()

    except:
        ac.log('SolWeatherApp: init main app: ' + traceback.format_exc())
    bPause=False

########################################################


def appSizeUp(*args):
    global gAppsize, settingsFilePath, documentSectionGeneral
    gAppsize += 0.05
    if gAppsize>4.0:
        gAppsize = 4.0
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiSize', str(gAppsize), True)
    appReadSettings()
    appSetPositionsAndSizes()
    appSetPositionsAndSizesPlan()

def appSizeDn(*args):
    global gAppsize, settingsFilePath, documentSectionGeneral
    gAppsize -= 0.05
    if gAppsize<0.1:
        gAppsize = 0.1
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiSize', str(gAppsize), True)
    appReadSettings()
    appSetPositionsAndSizes()
    appSetPositionsAndSizesPlan()

def appSizeHigher(*args):
    global gAppsize, settingsFilePath, documentSectionGeneral, gExtraHeight
    gExtraHeight -= 20
    if gExtraHeight:
        gExtraHeight==1000
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiExtraHeight', str(gExtraHeight), True)
    appReadSettings()
    appSetPositionsAndSizes()
    appSetPositionsAndSizesPlan()

def appSizeLower(*args):
    global gAppsize, settingsFilePath, documentSectionGeneral, gExtraHeight
    gExtraHeight += 20
    if gExtraHeight<-800:
        gExtraHeight==-800
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiExtraHeight', str(gExtraHeight), True)
    appReadSettings()
    appSetPositionsAndSizes()
    appSetPositionsAndSizesPlan()

def appMinimalToggle(*args):
    global app, ps_app, settingsFilePath, gMinimal, documentSectionGeneral, lbTime, lbDate, lbMult
    gMinimal = not gMinimal
    ac.setText(lbTime,'')
    ac.setText(lbDate,'')
    ac.setText(lbMult,'')
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiMinimal', str(gMinimal), True)
    appReadSettings()
    appSetPositionsAndSizes()
    appSetPositionsAndSizesPlan()

def appToggleTransparency(*args):
    global app, ps_app, settingsFilePath, gHideBG, documentSectionGeneral
    gHideBG = not gHideBG
    if gHideBG==True:
        ac.drawBorder(app, 0)
        ac.drawBorder(ps_app, 0)
        ac.setBackgroundOpacity(app, 0)
        ac.setBackgroundOpacity(ps_app, 0)
    else:
        ac.drawBorder(app, 1)
        ac.drawBorder(ps_app, 1)
        ac.setBackgroundOpacity(app, 0.5)
        ac.setBackgroundOpacity(ps_app, 0.5)
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiHideBG', str(gHideBG), True)
    appReadSettings()
    appSetPositionsAndSizes()
    appSetPositionsAndSizesPlan()

def ps_LIT_MULT(*args):
    global spinner_LIT_MULT, bPause, gLIT_MULT
    if not bPause:
        gLIT_MULT = str(    float(ac.getValue(spinner_LIT_MULT))/100.0   )
        appApplyTrackConfig()

def ps_CAR_LIGHTS_LIT_MULT(*args):
    global spinner_CAR_LIGHTS_LIT_MULT, bPause, gCAR_LIGHTS_LIT_MULT
    if not bPause:
        gCAR_LIGHTS_LIT_MULT = str(    float(ac.getValue(spinner_CAR_LIGHTS_LIT_MULT))/100.0   )
        appApplyTrackConfig()

def ps_SPECULAR_MULT(*args):
    global spinner_SPECULAR_MULT, bPause, gSPECULAR_MULT
    if not bPause:
        gSPECULAR_MULT = str(    float(ac.getValue(spinner_SPECULAR_MULT))/100.0   )
        appApplyTrackConfig()

#############################################################################################

def appShowMain(*args):
    global app, bVisibleMain
    try:
        bVisibleMain = ac.ext_isAppVisible("Sol Weather")
    except:
        bVisibleMain = bVisibleMain # do noting if patch with needed function is not availble
    bVisibleMain = not bVisibleMain
    ac.setVisible(app, bVisibleMain)


def appShowPlan(*args):
    global ps_app, bVisiblePlan
    try:
        bVisiblePlan = ac.ext_isAppVisible("Sol Plan Selector")
    except:
        bVisiblePlan = bVisiblePlan # do noting if patch with needed function is not availble
    bVisiblePlan = not bVisiblePlan
    ac.setVisible(ps_app, bVisiblePlan)

def appCloseMain(*args):
    global app, bVisibleMain
    bVisibleMain=False
    ac.setVisible(app,0)

def appClosePlan(*args):
    global ps_app, bNewPlan, bVisiblePlan
    if bNewPlan:
        bNewPlan=False
        appSetPositionsAndSizesPlan()
    bVisiblePlan=False
    ac.setVisible(ps_app,0)

def appOnActivatedMain(*args):
    global bVisibleMain
    bVisibleMain=True

def appOnActivatedPlan(*args):
    global bVisiblePlan
    bVisiblePlan=True

def appOnClosedMain(*args):
    global bVisibleMain, app
    bVisibleMain=False
def appOnClosedPlan(*args):
    global ps_app, bVisiblePlan
    bVisiblePlan=False

def ps_settingstoggle(*args):
    global btnSettingsToggle, gShowSettings, settingsFilePath, documentSectionGeneral
    if ac.getText(btnSettingsToggle) == 'v':
        gShowSettings = True
        ac.setText(btnSettingsToggle, '^')
    else:
        ac.setText(btnSettingsToggle, 'v')
        gShowSettings = False
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiShowMoreSettingsOnPlan',str(gShowSettings), True)
    appSetPositionsAndSizesPlan()

def appSeasonToggle(*args):
    global checkbox_Seasons, gSeasonsActive, sPatchWeatherFXini, sPatchTrackAdjust
    if str(args[1])=='1':
    # if int(ac.getValue(checkbox_Seasons))==1:
        gSeasonsActive = True
        appWriteCFGValue(sPatchTrackAdjust , 'SEASONS'      , 'ALLOW_ADJUSTMENTS'             , '1', True)
        appWriteCFGValue(sPatchWeatherFXini, 'MISCELLANEOUS', 'USE_TRACK_SEASONAL_ADJUSTMENTS', '1', True)
    else:
        gSeasonsActive = False
        appWriteCFGValue(sPatchTrackAdjust , 'SEASONS'      , 'ALLOW_ADJUSTMENTS'             , '0', True)
        appWriteCFGValue(sPatchWeatherFXini, 'MISCELLANEOUS', 'USE_TRACK_SEASONAL_ADJUSTMENTS', '0', True)

##############################################################################

def appCreateMyself():
    global app, lbWeat
    global timeOffsetMinusSmallButton, timeOffsetPlusSmallButton, timeOffsetMinusBigButton, timeOffsetPlusBigButton, timeOffsetMinusHugeButton, timeOffsetPlusHugeButton, timeOffsetMinusPlayButton, timeOffsetPlusPlayButton
    global timeOffsetMinusSmallerButton, timeOffsetPlusSmallerButton, timeOffsetMinusPlaySmallButton, timeOffsetPlusPlaySmallButton
    global timeOffsetMinusMonthButton, timeOffsetPlusMonthButton, timeOffsetMinusYearButton, timeOffsetPlusYearButton
    global timeAccelDownButton, timeAccelUpButton, timeAccelUpFasterButton, timeAccelDownFasterButton, timeAccelResetButton, timeAccelStopButton
    global btnShowSolSelector, btnAppClose
    global gHideBG, gAppShowOnStart, gButtonH, gExtraHeight
    global gUseSymbols, bUseCustomFont
    global lbTime, lbDate, lbMult, lbChng, btnAppSizeUp, btnAppSizeDn, btnTranspToggleAPP, btnMinimalToggle, btnAppSizeHigher, btnAppSizeLower
    try:
        app = ac.newApp("Sol Weather")
        ac.addOnAppDismissedListener(app, appOnClosedMain)
        ac.addOnAppActivatedListener(app, appOnActivatedMain)
        ac.setTitle(app, "")

        lbWeat = ac.addLabel(app, "") # weather debug text lbWeat
        lbChng = ac.addLabel(app, "") # time changing
        lbTime = ac.addLabel(app, "") # time
        lbDate = ac.addLabel(app, "") # date
        lbMult = ac.addLabel(app, "") # time multiplier

        btnAppClose        = ac.addButton(app, "x")  # app close
        btnAppSizeUp       = ac.addButton(app, "")
        btnAppSizeDn       = ac.addButton(app, "")
        btnAppSizeHigher   = ac.addButton(app, "")
        btnAppSizeLower    = ac.addButton(app, "")
        btnTranspToggleAPP = ac.addButton(app, "")
        btnMinimalToggle   = ac.addButton(app, "")

        btnShowSolSelector = ac.addButton(app, "") # >>> Sol weather plans button
        timeOffsetPlusBigButton = ac.addButton(app, "") # all the time buttons
        timeOffsetMinusBigButton = ac.addButton(app, "")
        timeOffsetPlusSmallButton = ac.addButton(app, "")
        timeOffsetMinusSmallButton = ac.addButton(app, "")
        timeOffsetPlusSmallerButton = ac.addButton(app, "")
        timeOffsetMinusSmallerButton = ac.addButton(app, "")
        timeOffsetPlusYearButton = ac.addButton(app, "")
        timeOffsetMinusYearButton = ac.addButton(app, "")
        timeOffsetPlusMonthButton = ac.addButton(app, "")
        timeOffsetMinusMonthButton = ac.addButton(app, "")
        timeOffsetPlusHugeButton = ac.addButton(app, "")
        timeOffsetMinusHugeButton = ac.addButton(app, "")
        timeOffsetMinusPlaySmallButton = ac.addButton(app, "")
        timeOffsetMinusPlayButton = ac.addButton(app, "")
        timeOffsetPlusPlaySmallButton = ac.addButton(app, "")
        timeOffsetPlusPlayButton = ac.addButton(app, "")
        timeAccelDownButton = ac.addButton(app, "")
        timeAccelDownFasterButton = ac.addButton(app, "")
        timeAccelUpButton = ac.addButton(app, "")
        timeAccelUpFasterButton = ac.addButton(app, "")
        timeAccelStopButton = ac.addButton(app, "p")  # stop/pause
        timeAccelResetButton = ac.addButton(app, "r")  # reset

        ac.addOnClickedListener(btnShowSolSelector              , appShowPlan)
        ac.addOnClickedListener(btnAppClose                     , appCloseMain)
        ac.addOnClickedListener(btnAppSizeUp                    , appSizeUp)
        ac.addOnClickedListener(btnAppSizeDn                    , appSizeDn)
        ac.addOnClickedListener(btnAppSizeHigher                , appSizeHigher)
        ac.addOnClickedListener(btnAppSizeLower                 , appSizeLower)

        ac.addOnClickedListener(btnTranspToggleAPP              , appToggleTransparency)
        ac.addOnClickedListener(btnMinimalToggle                , appMinimalToggle)
        ac.addOnClickedListener(timeOffsetPlusBigButton         , timeOffsetPlusBig)
        ac.addOnClickedListener(timeOffsetMinusBigButton        , timeOffsetMinusBig)
        ac.addOnClickedListener(timeOffsetPlusSmallButton       , timeOffsetPlusSmall)
        ac.addOnClickedListener(timeOffsetMinusSmallButton      , timeOffsetMinusSmall)
        ac.addOnClickedListener(timeOffsetPlusSmallerButton     , timeOffsetPlusSmaller)
        ac.addOnClickedListener(timeOffsetMinusSmallerButton    , timeOffsetMinusSmaller)
        ac.addOnClickedListener(timeOffsetPlusYearButton        , timeOffsetPlusYear)
        ac.addOnClickedListener(timeOffsetMinusYearButton       , timeOffsetMinusYear)
        ac.addOnClickedListener(timeOffsetPlusMonthButton       , timeOffsetPlusMonth)
        ac.addOnClickedListener(timeOffsetMinusMonthButton      , timeOffsetMinusMonth)
        ac.addOnClickedListener(timeOffsetPlusHugeButton        , timeOffsetPlusDay)
        ac.addOnClickedListener(timeOffsetMinusHugeButton       , timeOffsetMinusDay)
        ac.addOnClickedListener(timeOffsetMinusPlaySmallButton  , timeOffsetMinusPlaySmall)
        ac.addOnClickedListener(timeOffsetMinusPlayButton       , timeOffsetMinusPlay)
        ac.addOnClickedListener(timeOffsetPlusPlaySmallButton   , timeOffsetPlusPlaySmall)
        ac.addOnClickedListener(timeOffsetPlusPlayButton        , timeOffsetPlusPlay)
        ac.addOnClickedListener(timeAccelDownButton             , timeAccelDown)
        ac.addOnClickedListener(timeAccelDownFasterButton       , timeAccelDownFaster)
        ac.addOnClickedListener(timeAccelUpButton               , timeAccelUp)
        ac.addOnClickedListener(timeAccelUpFasterButton         , timeAccelUpFaster)
        ac.addOnClickedListener(timeAccelStopButton             , timeAccelStop)
        ac.addOnClickedListener(timeAccelResetButton            , timeAccelReset)
    except:
        ac.log('SolWeatherApp: init main app: ' + traceback.format_exc())
        ac.console('SolWeatherApp: init main app: ' + traceback.format_exc())

def appSetPositionsAndSizes():
    global app, lbWeat
    global timeOffsetMinusSmallButton, timeOffsetPlusSmallButton, timeOffsetMinusBigButton, timeOffsetPlusBigButton, timeOffsetMinusHugeButton, timeOffsetPlusHugeButton, timeOffsetMinusPlayButton, timeOffsetPlusPlayButton
    global timeOffsetMinusSmallerButton, timeOffsetPlusSmallerButton, timeOffsetMinusPlaySmallButton, timeOffsetPlusPlaySmallButton
    global timeOffsetMinusMonthButton, timeOffsetPlusMonthButton, timeOffsetMinusYearButton, timeOffsetPlusYearButton
    global timeAccelDownButton, timeAccelUpButton, timeAccelUpFasterButton, timeAccelDownFasterButton, timeAccelResetButton, timeAccelStopButton
    global btnShowSolSelector
    global gAppsize, gHideBG, gAppShowOnStart, gButtonH, gButtonsAboveDateTime, gExtraHeight
    global gUseSymbols, sPatchVersion
    global lbTime, lbDate, lbMult, lbChng, btnAppSizeUp, btnAppSizeDn, gMinimal, btnTranspToggleAPP, btnAppClose, btnMinimalToggle, btnAppSizeHigher, btnAppSizeLower
    global gOffsetSmaller, gOffsetSmall, gOffsetBig, sCustomFontName, bUseCustomFont
    try:
        if bUseCustomFont:
            ac.setCustomFont(lbWeat                          , sCustomFontName, 0 , 0)
            ac.setCustomFont(lbTime                          , sCustomFontName, 0 , 0)
            ac.setCustomFont(lbDate                          , sCustomFontName, 0 , 0)
            ac.setCustomFont(lbMult                          , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnShowSolSelector              , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnAppClose                     , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnAppSizeUp                    , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnAppSizeDn                    , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnAppSizeHigher                , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnAppSizeLower                 , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnTranspToggleAPP              , sCustomFontName, 0 , 0)
            ac.setCustomFont(btnMinimalToggle                , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusBigButton         , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusBigButton        , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusSmallButton       , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusSmallButton      , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusSmallerButton     , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusSmallerButton    , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusYearButton        , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusYearButton       , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusMonthButton       , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusMonthButton      , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusHugeButton        , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusHugeButton       , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusPlaySmallButton  , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetMinusPlayButton       , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusPlaySmallButton   , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeOffsetPlusPlayButton        , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeAccelDownButton             , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeAccelDownFasterButton       , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeAccelUpButton               , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeAccelUpFasterButton         , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeAccelStopButton             , sCustomFontName, 0 , 0)
            ac.setCustomFont(timeAccelResetButton            , sCustomFontName, 0 , 0)

        if gHideBG==True:
            ac.drawBorder(app, 0)
            ac.setBackgroundOpacity(app, 0)
        else:
            ac.drawBorder(app, 1)
            ac.setBackgroundOpacity(app, 0.5)

        dx=25
        dy=gButtonH
        fntsize=12
        mult=1.0
        if gAppsize>=0.1 and gAppsize<=4.0:
            mult=gAppsize
            dx=dx*mult
            dy=dy*mult
            fntsize=fntsize*mult

        if bUseCustomFont:
            if (sCustomFontName=="Consolas"):
                fntsize *= 0.9
            if (sCustomFontName=="Digital-7"):
                fntsize *= 1.15
            if (sCustomFontName=="Ticking Timebomb BB"):
                fntsize *= 1.1
            if (sCustomFontName=="Strait"):
                fntsize *= 1.05

        xposBase=fntsize*1.25
        #yposBase=35
        yposBase=gButtonH/2 - 10

        ### window size
        ac.setIconPosition(app, 0, -20000)
        width  =     365*mult
        height =     int(200*mult + gButtonH + gExtraHeight)
        if height<25:
            height=25
            gExtraHeight = int(height - (200*mult + gButtonH))
        ac.setSize(app, width, max(20, height) )

        if gHideBG==True:
            if dy>25:
                ac.setSize(app, 365*mult, dy-10)
            else:
                ac.setSize(app, 365*mult, 25)

        ### labels time date accel + "wait for time to be changed"
        xpos = xposBase
        ypos = yposBase
        if gButtonsAboveDateTime==True:
            if dy>30:
                ypos -= fntsize*4.5 + dy
            else:
                ypos -= fntsize*5 + dy

        ac.setPosition(lbChng, xpos+dx*3.5, ypos-dy/2)
        ac.setPosition(lbTime, xpos-5, ypos)
        # ac.setPosition(btnShowSolSelector, -20000, ypos)
        if gMinimal==True:
            ac.setPosition(lbDate, -20000, ypos)
            ac.setPosition(lbMult, -20000, ypos)
        else:
            ac.setPosition(lbMult, xpos + dx*8+4, ypos+1)
            ac.setPosition(lbDate, xpos+dx*3+dx/3+2, ypos)

        ac.setSize(lbChng, dx, dy*4)
        ac.setSize(lbTime, dx, dy*4)
        ac.setSize(lbDate, dx, dy*4)
        ac.setSize(lbMult, dx, dy*4)
        ac.setFontSize(lbChng, fntsize-1)
        ac.setFontSize(lbTime, fntsize*1.5)
        ac.setFontSize(lbDate, fntsize*1.5)
        # ac.setFontSize(lbMult, fntsize*1.25)
        ac.setFontSize(lbMult, fntsize*1.5)

        ### show plan button + weather debug text ################
        if gButtonsAboveDateTime==True:
            ypos = yposBase + fntsize
            ypos = yposBase - fntsize*4 + 3
        else:
            ypos = yposBase + dy*2 + fntsize*3

        ypos+=fntsize*2.5
        if gButtonsAboveDateTime==True:
            ac.setPosition(btnShowSolSelector, xpos+fntsize-2, ypos-dy*5.5)     # >>> button
        else:
            if gMinimal==True:
                ac.setPosition(btnShowSolSelector, -20000, ypos)     # >>> button
            else:
                ac.setPosition(btnShowSolSelector, xpos+fntsize-2, ypos)     # >>> button
        ac.setSize(btnShowSolSelector, dx+8, fntsize*1.75)
        ac.setFontSize(btnShowSolSelector, fntsize+2)

        ac.setPosition(lbWeat, xpos+fntsize, ypos)
        ac.setFontSize(lbWeat, fntsize+2)

        ### all the buttons #######################################
        xpos = xposBase + dx/2
        ypos = yposBase + fntsize*3
        if gButtonsAboveDateTime==True:
            ypos=yposBase - fntsize*8 - dy*2

        ac.setPosition(btnAppSizeUp, xpos-dx, ypos)    # app size +
        ac.setSize(btnAppSizeUp, dx-6, dy)
        ac.setFontSize(btnAppSizeUp, fntsize)
        ac.setBackgroundOpacity(btnAppSizeUp, 0)

        ac.setPosition(btnAppSizeDn, xpos-dx, ypos+dy)  # app size -
        ac.setSize(btnAppSizeDn, dx-6, dy)
        ac.setFontSize(btnAppSizeDn, fntsize)
        ac.setBackgroundOpacity(btnAppSizeDn, 0)

        ac.setPosition(btnAppSizeHigher, xpos-dx, ypos+dy*2+5)    # app height +
        ac.setSize(btnAppSizeHigher, dx-6, dy)
        ac.setFontSize(btnAppSizeHigher, fntsize)
        ac.setBackgroundOpacity(btnAppSizeHigher, 0)

        ac.setPosition(btnAppSizeLower, xpos-dx, ypos+dy*3+5)  # app height -
        ac.setSize(btnAppSizeLower, dx-6, dy)
        ac.setFontSize(btnAppSizeLower, fntsize)
        ac.setBackgroundOpacity(btnAppSizeLower, 0)

        ac.setPosition(timeOffsetPlusBigButton, xpos, ypos) # +6 btn
        ac.setSize(timeOffsetPlusBigButton, dx, dy)
        ac.setFontSize(timeOffsetPlusBigButton, fntsize)

        ypos+=dy
        ac.setPosition(timeOffsetMinusBigButton, xpos, ypos)
        ac.setSize(timeOffsetMinusBigButton, dx, dy)
        ac.setFontSize(timeOffsetMinusBigButton, fntsize)

        xpos+=dx
        ypos-=dy
        ac.setPosition(timeOffsetPlusSmallButton, xpos, ypos) # +45min
        ac.setSize(timeOffsetPlusSmallButton, dx, dy)
        ac.setFontSize(timeOffsetPlusSmallButton, fntsize)
        ypos+=dy
        ac.setPosition(timeOffsetMinusSmallButton, xpos, ypos)
        ac.setSize(timeOffsetMinusSmallButton, dx, dy)
        ac.setFontSize(timeOffsetMinusSmallButton, fntsize)

        xpos+=dx
        ypos-=dy
        ac.setPosition(timeOffsetPlusSmallerButton, xpos, ypos) # +5
        ac.setSize(timeOffsetPlusSmallerButton, dx, dy)
        ac.setFontSize(timeOffsetPlusSmallerButton, fntsize)
        ypos+=dy
        ac.setPosition(timeOffsetMinusSmallerButton, xpos, ypos)
        ac.setSize(timeOffsetMinusSmallerButton, dx, dy)
        ac.setFontSize(timeOffsetMinusSmallerButton, fntsize)

        xpos+=dx*1.5 #####################################
        ypos-=dy
        ac.setPosition(timeOffsetPlusYearButton, xpos, ypos) # +year
        ac.setSize(timeOffsetPlusYearButton, dx*2, dy)
        ac.setFontSize(timeOffsetPlusYearButton, fntsize)
        ypos+=dy
        ac.setPosition(timeOffsetMinusYearButton, xpos, ypos)
        ac.setSize(timeOffsetMinusYearButton, dx*2, dy)
        ac.setFontSize(timeOffsetMinusYearButton, fntsize)

        xpos+=dx*2
        ypos-=dy
        ac.setPosition(timeOffsetPlusMonthButton, xpos, ypos) # + month
        ac.setSize(timeOffsetPlusMonthButton, dx, dy)
        ac.setFontSize(timeOffsetPlusMonthButton, fntsize)
        ypos+=dy
        ac.setPosition(timeOffsetMinusMonthButton, xpos, ypos)
        ac.setSize(timeOffsetMinusMonthButton, dx, dy)
        ac.setFontSize(timeOffsetMinusMonthButton, fntsize)

        xpos+=dx
        ypos-=dy
        ac.setPosition(timeOffsetPlusHugeButton, xpos, ypos) # +day
        ac.setSize(timeOffsetPlusHugeButton, dx, dy)
        ac.setFontSize(timeOffsetPlusHugeButton, fntsize)
        ypos+=dy
        ac.setPosition(timeOffsetMinusHugeButton, xpos, ypos)
        ac.setSize(timeOffsetMinusHugeButton, dx, dy)
        ac.setFontSize(timeOffsetMinusHugeButton, fntsize)

        xpos+=dx*1.5-1   ##################################
        ypos-=dy
        ac.setPosition(timeOffsetMinusPlaySmallButton, xpos, ypos) # -7a/s
        ac.setSize(timeOffsetMinusPlaySmallButton, dx, dy)
        ac.setFontSize(timeOffsetMinusPlaySmallButton, fntsize)
        ypos+=dy
        ac.setPosition(timeOffsetMinusPlayButton, xpos, ypos) # -1d/s
        ac.setSize(timeOffsetMinusPlayButton, dx, dy)
        ac.setFontSize(timeOffsetMinusPlayButton, fntsize)

        xpos+=dx
        ypos-=dy
        ac.setPosition(timeOffsetPlusPlaySmallButton, xpos, ypos)
        ac.setSize(timeOffsetPlusPlaySmallButton, dx, dy)
        ac.setFontSize(timeOffsetPlusPlaySmallButton, fntsize)
        ypos+=dy
        ac.setPosition(timeOffsetPlusPlayButton, xpos, ypos)
        ac.setSize(timeOffsetPlusPlayButton, dx, dy)
        ac.setFontSize(timeOffsetPlusPlayButton, fntsize)

        xpos+=dx+5  ###########################################
        ypos-=dy
        ac.setPosition(timeAccelDownButton, xpos, ypos) # 1x
        ac.setSize(timeAccelDownButton, dx, dy)
        ac.setFontSize(timeAccelDownButton, fntsize)
        ypos+=dy
        ac.setPosition(timeAccelDownFasterButton, xpos, ypos) # 20x
        ac.setSize(timeAccelDownFasterButton, dx, dy)
        ac.setFontSize(timeAccelDownFasterButton, fntsize)

        xpos+=dx
        ypos-=dy
        ac.setPosition(timeAccelUpButton, xpos, ypos)
        ac.setSize(timeAccelUpButton, dx, dy)
        ac.setFontSize(timeAccelUpButton, fntsize)
        ypos+=dy
        ac.setPosition(timeAccelUpFasterButton, xpos, ypos)
        ac.setSize(timeAccelUpFasterButton, dx, dy)
        ac.setFontSize(timeAccelUpFasterButton, fntsize)

        xpos+=dx+6  ###########################################
        ypos-=dy
        ac.setPosition(timeAccelStopButton, xpos, ypos) # time pause btn
        ac.setSize(timeAccelStopButton, dx-4, dy)
        ac.setFontSize(timeAccelStopButton, fntsize-1)
        ac.setBackgroundOpacity(timeAccelStopButton, 0)
        #ac.drawBorder(timeAccelStopButton, 0)
        ypos+=dy
        ac.setPosition(timeAccelResetButton, xpos, ypos) # time reset btn
        ac.setSize(timeAccelResetButton, dx-4, dy)
        ac.setFontSize(timeAccelResetButton, fntsize-1)
        ac.setBackgroundOpacity(timeAccelResetButton, 0)
        #ac.drawBorder(timeAccelResetButton, 0)

        ypos+=dy
        if gButtonsAboveDateTime==True:
            ac.setPosition(btnAppClose, xpos, ypos-dy*3.5)           # x app close button
            ac.setPosition(btnTranspToggleAPP, xpos-dx-3, ypos-dy*3.5)  # toggle transp app
            ac.setPosition(btnMinimalToggle, xpos-dx*2-3, ypos-dy*3.5)  # toggle minimal
        else:
            ac.setPosition(btnAppClose, xpos, ypos+4)           # x app close button
            ac.setPosition(btnTranspToggleAPP, xpos-dx-3, ypos+4)  # toggle transp app
            ac.setPosition(btnMinimalToggle, xpos-dx*2-3, ypos+4)  # toggle minimal

        ac.setSize(btnAppClose, dx-4, fntsize*1.75)
        ac.setFontSize(btnAppClose, fntsize+2)

        ac.setSize(btnTranspToggleAPP, dx-4, fntsize*1.75)
        ac.setFontSize(btnTranspToggleAPP, fntsize)
        # ac.setBackgroundOpacity(btnTranspToggleAPP, 0)

        ac.setSize(btnMinimalToggle, dx-4, fntsize*1.75)
        ac.setFontSize(btnMinimalToggle, fntsize)
        # ac.setBackgroundOpacity(btnMinimalToggle, 0)

        ac.setText(btnMinimalToggle, "µ")
        # now buttons texts
        if gUseSymbols==True:
            ac.setText(btnAppSizeUp, "⌃")
            ac.setText(btnAppSizeDn, "⌄")
            ac.setText(btnAppSizeHigher, "⌃")
            ac.setText(btnAppSizeLower, "⌄")
            ac.setText(btnTranspToggleAPP, "⛶")
            # ac.setText(timeOffsetMinusBigButton, "6h▿") # -6
            # ac.setText(timeOffsetMinusSmallButton, "45▿") # -45
            # ac.setText(timeOffsetMinusSmallerButton, "5m▿") # -5
            ac.setText(timeOffsetMinusBigButton    , str( int( gOffsetBig/60/60)) + "h▿") # -6h
            ac.setText(timeOffsetMinusSmallButton  , str( int( gOffsetSmall/60))  + "▿") # -45min
            ac.setText(timeOffsetMinusSmallerButton, str( int( gOffsetSmaller/60))+ "m▿") # -5min
            ac.setText(timeOffsetPlusBigButton, "△") # +6
            ac.setText(timeOffsetPlusSmallButton, "∆") # +45
            ac.setText(timeOffsetPlusSmallerButton, "▵") # +5
            ac.setText(timeOffsetPlusYearButton, "▲") # +yr
            ac.setText(timeOffsetMinusYearButton, "y▾") # -yr
            ac.setText(timeOffsetPlusMonthButton, "⏶") # +month
            ac.setText(timeOffsetMinusMonthButton, "m▾") # -month
            ac.setText(timeOffsetPlusHugeButton, "▴") # +day
            ac.setText(timeOffsetMinusHugeButton, "d▾") # -day
            ac.setText(timeOffsetPlusPlaySmallButton, "1d▸")  # +1a/s
            ac.setText(timeOffsetMinusPlaySmallButton, "◂") # -1a/s
            ac.setText(timeOffsetPlusPlayButton, "7d▸")       # +7a/s
            ac.setText(timeOffsetMinusPlayButton, "◀")      # -7a/s
            ac.setText(timeAccelUpButton, "1x▹")         # +1x
            ac.setText(timeAccelDownButton, "◃")       # -1x
            ac.setText(timeAccelUpFasterButton, "20x▹")   # +20x
            ac.setText(timeAccelDownFasterButton, "◁") # -20x
            ac.setText(timeAccelStopButton, "⏸")  # stop/pause
            ac.setText(timeAccelResetButton, "↩")  # reset
            if gButtonH>=25:
                # ac.setText(timeOffsetMinusBigButton, "▽\n6h") # -6
                # ac.setText(timeOffsetMinusSmallButton, "∇\n45m") # -45min
                # ac.setText(timeOffsetMinusSmallerButton, "▿\n5m") # -5
                ac.setText(timeOffsetMinusBigButton    , "▽\n" + str( int( gOffsetBig/60/60)) +"h") # -6h
                ac.setText(timeOffsetMinusSmallButton  , "∇\n" + str( int( gOffsetSmall/60)) + "m") # -45min
                ac.setText(timeOffsetMinusSmallerButton, "▿\n" + str( int( gOffsetSmaller/60)) + "m") # -5min
                ac.setText(timeOffsetMinusYearButton, "▼\nyrs") # -yr
                ac.setText(timeOffsetMinusMonthButton, "⏷\nmth") #- mnth
                ac.setText(timeOffsetMinusHugeButton, "▾\ndys") # -day
                ac.setText(timeOffsetPlusPlaySmallButton, "▸\n1d")  # +a/s
                ac.setText(timeOffsetPlusPlayButton, "▶\n7d") # +7a/s
                ac.setText(timeAccelUpButton, "▹\n1x") # +1x
                ac.setText(timeAccelUpFasterButton, "▷\n20x")  # +20x
        else:
            ac.setText(btnAppSizeUp, "+")
            ac.setText(btnAppSizeDn, "-")
            ac.setText(btnAppSizeHigher, "-")
            ac.setText(btnAppSizeLower, "+")
            ac.setText(btnTranspToggleAPP, "[ ]")
            ac.setText(timeOffsetPlusBigButton, "+")       # +6
            ac.setText(timeOffsetMinusBigButton, "-6h")    # -6
            ac.setText(timeOffsetPlusSmallButton, "+")     # +45
            ac.setText(timeOffsetMinusSmallButton, "-45")  # -45
            ac.setText(timeOffsetPlusSmallerButton, "+")   # +5
            ac.setText(timeOffsetMinusSmallerButton, "-5m") # -5
            ac.setText(timeOffsetPlusYearButton, "+")  # +yr
            ac.setText(timeOffsetMinusYearButton, "-yr") # -yr
            ac.setText(timeOffsetPlusMonthButton, "+") # +month
            ac.setText(timeOffsetMinusMonthButton, "-mon") # -month
            ac.setText(timeOffsetPlusHugeButton, "+") # +day
            ac.setText(timeOffsetMinusHugeButton, "-day") # -day
            ac.setText(timeOffsetPlusPlaySmallButton, "+1d")  # +a/s
            ac.setText(timeOffsetMinusPlaySmallButton, "-") # -a/s
            ac.setText(timeOffsetPlusPlayButton, "+7d")       # +7a/s
            ac.setText(timeOffsetMinusPlayButton, "-")      # -7a/s
            ac.setText(timeAccelUpButton, "1x")         # +1x
            ac.setText(timeAccelDownButton, "-")       # -1x
            ac.setText(timeAccelUpFasterButton, "20x")   # +20x
            ac.setText(timeAccelDownFasterButton, "-") # -20x
            ac.setText(timeAccelStopButton, "p")  # stop/pause
            ac.setText(timeAccelResetButton, "r")  # reset
            if gButtonH>=25:
                ac.setText(timeOffsetMinusBigButton, "-\n6h")     # -6
                ac.setText(timeOffsetMinusSmallButton, "-\n45m")  # -45min
                ac.setText(timeOffsetMinusSmallerButton, "-\n5m") # -5
                ac.setText(timeOffsetMinusYearButton, "-\nyrs") # -yr
                ac.setText(timeOffsetPlusMonthButton, "+") # +mnt
                ac.setText(timeOffsetMinusMonthButton, "-\nmth") #- mnth
                ac.setText(timeOffsetPlusHugeButton, "+") # +day
                ac.setText(timeOffsetMinusHugeButton, "-\ndys") # -day
                ac.setText(timeOffsetMinusPlayButton, "-") # -7a/s
                ac.setText(timeOffsetPlusPlaySmallButton, "+\n1d")  # +a/s
                ac.setText(timeOffsetPlusPlayButton, "+\n7d") # +7a/s
                ac.setText(timeAccelUpButton, "+\n1x") # +1x
                ac.setText(timeAccelUpFasterButton, "+\n20x")  # +20x
    except:
        ac.log('SolWeatherApp error: ' + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())
        ac.setText(lbChng, traceback.format_exc())

############################# Plan Selector ##################################
def appCreatePlanSelector():
    global ps_app, spinner_ps_plan, checkbox_ps_plan_enable, lb_spinner_ps_plan, lbWeatherinfo
    global weathers, checkbox_ps_weather, lbDesc0, lbDesc1, lbDesc2
    global gAppsize, gAppShowOnStart, spinnerGlow, lbGlow, btnSettingsToggle
    global lbCar, sTrack, sLayout, checkbox_Tracklights, spinnerRangeTrackLights, gButtonH, gTracklightsRange, gTracklightsGLOW, spinnerRangeHeadLights, spinnerSmokeAmount, spinnerSmokeThickness
    global buttonResetWeather, lbRang, gUseSymbols, lbChng, gUseScaling
    global lbNewPlan, btnNewPlan, btnNewRandWeather, btnNewPlansave, lbNewPlanWeatherDuration, lbNewPlanRandom, edtFilename, sNewWeather, sNewWeatherClick
    global spinnerTimeHolding, spinnerTimeChanging, spinnerTemperature_ambient, spinnerTemperature_road, spinnerWind_direction, spinnerWind_speed, chkBoxRandomTH, chkBoxRandomTC, chkBoxRandomAT, chkBoxRandomRT, chkBoxRandomWD, chkBoxRandomWS
    global bPause, btnFileCar, btnFileCar2, btnFileTrack, btnFileTrack2, night__brightness_adjust, chkBoxPlanLoop
    global checkbox_AutoRain, checkbox_AutoWipers, spinnerRain, spinnerWipers, spinnerSound, checkbox_ManualRain, checkbox_ManualWipers
    global spinnerRainV, spinnerWipersV, spinnerSoundV, btnPlanClose, weather_orig, btnToggleTransparencyPS
    global sol_weather_plan, gMaximumValues, gSteps, sPatchVersion, gOrigSmokeAmount, gOrigSmokeThickness
    global spinnerNightBright, lbNigh, spinner_LIT_MULT, spinner_CAR_LIGHTS_LIT_MULT, spinner_SPECULAR_MULT, lbLIT_MULT, lbCAR_LIGHTS_LIT_MULT, lbSPECULAR_MULT
    global gLIT_MULT, gCAR_LIGHTS_LIT_MULT, gSPECULAR_MULT, checkbox_Seasons, gSeasonsActive
    try:
        # read current game conditions
        temperature_ambient = str(acinfo.physics.airTemp)
        temperature_road = str(acinfo.physics.roadTemp)
        wind_direction = str(round(acinfo.graphics.windDirection,1))
        wind_speed = str(random.randint(2, 15) if acinfo.graphics.windSpeed == 0 else round(acinfo.graphics.windSpeed,1))

        ps_app = ac.newApp("Sol Plan Selector")
        ac.addOnAppActivatedListener(ps_app, appOnActivatedPlan)
        ac.addOnAppDismissedListener(ps_app, appOnClosedPlan)

        # new plan new weather stuff
        btnNewPlan = ac.addButton(ps_app, ">")
        btnPlanClose = ac.addButton(ps_app, "x")
        if gUseSymbols==True:
            btnToggleTransparencyPS = ac.addButton(ps_app, "⛶")
        else:
            btnToggleTransparencyPS = ac.addButton(ps_app, "[ ]")
        btnNewPlansave = ac.addButton(ps_app, "Save new plan")
        ac.setFontColor(btnNewPlansave, 0.25,1,0.25,1)
        btnNewRandWeather = ac.addButton(ps_app, 'add random')

        chkBoxPlanLoop = ac.addCheckBox(ps_app, "loop new plan, don't hold last weather")

        #lbNewPlan = ac.addLabel(ps_app, "weather a\nweather b\nweather c") # new plan text
        lbNewPlan = ac.addButton(ps_app, "weather a\nweather b\nweather c") # new plan text use button bc of border
        ac.setFontAlignment(lbNewPlan, 'left')

        lbNewPlanWeatherDuration = ac.addLabel(ps_app, sNewWeather)
        lbNewPlanRandom = ac.addLabel(ps_app, sNewWeatherRandom)

        # sNewWeatherRandom
        spinnerTimeHolding = ac.addSpinner(ps_app, "")
        spinnerTimeChanging = ac.addSpinner(ps_app, "")
        spinnerTemperature_ambient = ac.addSpinner(ps_app, "")
        spinnerTemperature_road = ac.addSpinner(ps_app, "")
        spinnerWind_direction = ac.addSpinner(ps_app, "")
        spinnerWind_speed = ac.addSpinner(ps_app, "")
        chkBoxRandomTH = ac.addCheckBox(ps_app,"")
        chkBoxRandomTC = ac.addCheckBox(ps_app,"")
        chkBoxRandomAT = ac.addCheckBox(ps_app,"")
        chkBoxRandomRT = ac.addCheckBox(ps_app,"")
        chkBoxRandomWD = ac.addCheckBox(ps_app,"")
        chkBoxRandomWS = ac.addCheckBox(ps_app,"")

        ac.addOnClickedListener(btnNewPlan, ps_newplan)
        ac.addOnClickedListener(btnPlanClose, appClosePlan)
        ac.addOnClickedListener(btnToggleTransparencyPS, appToggleTransparency)
        ac.addOnClickedListener(btnNewPlansave, ps_newplan_save)
        ac.addOnClickedListener(btnNewRandWeather, ps_newrndweather)
        ac.addOnCheckBoxChanged(chkBoxPlanLoop, chkboxchangedPlanLoop)

        ac.addOnValueChangeListener(spinnerTimeHolding, onSpinnerTime)
        ac.addOnValueChangeListener(spinnerTimeChanging, onSpinnerTime)
        ac.addOnValueChangeListener(spinnerWind_speed, onSpinnerTime)
        ac.addOnCheckBoxChanged(chkBoxRandomTH, chkboxchangedTH)
        ac.addOnCheckBoxChanged(chkBoxRandomTC, chkboxchangedTC)
        ac.addOnCheckBoxChanged(chkBoxRandomAT, chkboxchangedAT)
        ac.addOnCheckBoxChanged(chkBoxRandomRT, chkboxchangedRT)
        ac.addOnCheckBoxChanged(chkBoxRandomWD, chkboxchangedWD)
        ac.addOnCheckBoxChanged(chkBoxRandomWS, chkboxchangedWS)

        ac.setRange(spinnerTimeHolding, 0, 1000)
        ac.setStep(spinnerTimeHolding, 1)
        ac.setValue(spinnerTimeHolding, 5)

        ac.setRange(spinnerTimeChanging, 0, 1000)
        ac.setStep(spinnerTimeChanging, 1)
        ac.setValue(spinnerTimeChanging, 5)

        ac.setRange(spinnerTemperature_ambient, 0, 60)
        ac.setStep(spinnerTemperature_ambient, 1)
        ac.setValue(spinnerTemperature_ambient, int(float(temperature_ambient)))

        ac.setRange(spinnerTemperature_road, 0, 60)
        ac.setStep(spinnerTemperature_road, 1)
        ac.setValue(spinnerTemperature_road, int(float(temperature_road)))

        ac.setRange(spinnerWind_direction, 0, 360)
        ac.setStep(spinnerWind_direction, 1)
        ac.setValue(spinnerWind_direction, int(float(wind_direction)))

        ac.setRange(spinnerWind_speed, 0, 100)
        ac.setStep(spinnerWind_speed, 1)
        ac.setValue(spinnerWind_speed, int(float(wind_speed)))

        edtFilename = ac.addTextInput(ps_app, 'plan.lua')
        ac.setText(edtFilename, 'plan.lua')

        ### enable dyn weather label
        checkbox_ps_plan_enable = ac.addCheckBox(ps_app, '')
        ac.addOnCheckBoxChanged(checkbox_ps_plan_enable, ps_plan_enable)
        lbDesc0 = ac.addLabel(ps_app, 'Enable dynamic weather (permanent)\n-use a plan:\n\n-or force a weather:')

        lbWeatherinfo = ac.addLabel(ps_app, '')
        ac.setFontColor(lbWeatherinfo,0.25,1,0.25,1)

        ### reset weather button
        buttonResetWeather = ac.addButton(ps_app, "reset weather")
        ac.addOnClickedListener(buttonResetWeather, ps_reset_weather)

        ### show more settings   button
        btnSettingsToggle = ac.addButton(ps_app, 'v')
        if gShowSettings==True:
            ac.setText(btnSettingsToggle, '^')
        ac.addOnClickedListener(btnSettingsToggle, ps_settingstoggle)
        ac.drawBorder(btnSettingsToggle, 0)

        ### plan spinner
        spinner_ps_plan = ac.addSpinner(ps_app, "")
        ac.addOnValueChangeListener(spinner_ps_plan, ps_set_plan)
        ac.setStep(spinner_ps_plan, 1)

        ### plans label, ie "Demo.lua" or other filename from
        # assettocorsa\extension\weather-controllers\sol\weather_plans\
        lb_spinner_ps_plan = ac.addLabel(ps_app, "plan.lua")
        # ac.setFontColor(lb_spinner_ps_plan,0.25,1,0.25,1)
        ac.setFontColor(lb_spinner_ps_plan,0.5,0.5,0.5,0.5)

        spinnerRain = ac.addSpinner(ps_app, "")
        spinnerWipers = ac.addSpinner(ps_app, "")
        spinnerSound = ac.addSpinner(ps_app, "")
        checkbox_AutoRain = ac.addCheckBox( ps_app, 'rain ctrl off' ) # autorain + autowipers
        checkbox_AutoWipers = ac.addCheckBox( ps_app, 'wipers ctrl off' )
        checkbox_ManualRain = ac.addCheckBox( ps_app, '' ) # autorain + autowipers
        checkbox_ManualWipers = ac.addCheckBox( ps_app, '' )

        ac.setStep(spinnerRain, 5)
        ac.setStep(spinnerWipers, 1)
        ac.setStep(spinnerSound, 5)
        ac.setRange(spinnerRain, 0, 100)
        ac.setRange(spinnerWipers, 0, 4)
        ac.setRange(spinnerSound, 0, 100)

        ac.setValue(spinnerRain, int(spinnerRainV))
        ac.setValue(spinnerWipers, int(spinnerWipersV))
        ac.setValue(spinnerSound, int(spinnerSoundV))

        s1=''
        s2=''
        try:
            for weather in weathers:
                if gUseSymbols==True:
                    checkbox_ps_weather[weather] = ac.addCheckBox( ps_app, weathersNames[weathers.index(weather)] )
                else:
                    checkbox_ps_weather[weather] = ac.addCheckBox( ps_app, weathers[weathers.index(weather)] )
                ac.addOnCheckBoxChanged(checkbox_ps_weather[weather], ps_force_weather)
                if gUseSymbols==True:
                    s2 = s2 + weathersNames[weathers.index(weather)] +'\n'
                else:
                    s2 = s2 + weathers[weathers.index(weather)] + '\n'
                if (weathers.index(weather) + 1) % 16 == 0:
                    if s1=='':
                        s1=s2 # magic
                        s2=''
            lbDesc1 = ac.addLabel(ps_app, s1)
            lbDesc2 = ac.addLabel(ps_app, s2)
        except:
            ac.log('SolWeatherApp: init plan window: ' + traceback.format_exc())


        spinnerRangeTrackLights = ac.addSpinner(ps_app, "") # tracklights range spinner
        spinnerGlow = ac.addSpinner(ps_app, "")             # tracklights glow spinner
        spinnerRangeHeadLights = ac.addSpinner(ps_app, "")      # car headlights range spinner
        spinnerSmokeAmount = ac.addSpinner(ps_app, "")          # smoke multiplier
        spinnerSmokeThickness = ac.addSpinner(ps_app, "")       # smoke thickness

        if gLoadSaveTracklightsPerTrack==True:
            lbGlow = ac.addLabel(ps_app, "per track                              glow  %")
        else:
            lbGlow = ac.addLabel(ps_app, "glow  %")
        # lbCar = ac.addLabel(ps_app, '-car-.ini')
        lbCar = ac.addLabel(ps_app, '-car-.ini\nheadlights range %\n\nsmoke strength %\n\nsmoke thickness %')
        lbRang = ac.addLabel(ps_app, "-tracklights-")

        checkbox_Tracklights = ac.addCheckBox(ps_app, "")
        ac.setValue(checkbox_Tracklights, 1)
        btnFileTrack = ac.addButton(ps_app, '=')            # track config editor btn
        btnFileTrack2 = ac.addButton(ps_app, '=²')            # track config editor btn
        btnFileCar = ac.addButton(ps_app, '=')              # cae config
        btnFileCar2 = ac.addButton(ps_app, '=')              # cae config

        ac.setRange(spinnerRangeTrackLights, 0, gMaximumValues)
        ac.setRange(spinnerGlow, 0, gMaximumValues)
        ac.setRange(spinnerRangeHeadLights, 0, gMaximumValues*2)

        ac.setStep(spinnerRangeTrackLights, gSteps)
        ac.setStep(spinnerGlow, gSteps)
        ac.setStep(spinnerRangeHeadLights, gSteps*2)

        ac.setValue(spinnerRangeTrackLights, gTracklightsRange)
        ac.setValue(spinnerGlow, gTracklightsGLOW)
        ac.setValue(spinnerRangeHeadLights, gOrigHeadLightRange)

        if sVerCode!='' and int(sVerCode) <= 832:
            ac.setRange(spinnerSmokeAmount, 0, max(int(gOrigSmokeAmount), gMaximumValues*6) )
            ac.setStep(spinnerSmokeAmount, 5)
        else:
            ac.setRange(spinnerSmokeAmount, 0, 20000)   # gMaximumValues*6*10 10000
            ac.setStep(spinnerSmokeAmount, 50)
        ac.setValue(spinnerSmokeAmount, gOrigSmokeAmount)

        ac.setRange(spinnerSmokeThickness, 0, 20000) # 10000
        ac.setStep(spinnerSmokeThickness, 50)
        ac.setValue(spinnerSmokeThickness, gOrigSmokeThickness)

        ac.addOnValueChangeListener(spinnerRain, ps_rain)
        ac.addOnValueChangeListener(spinnerWipers, ps_wipers)
        ac.addOnValueChangeListener(spinnerSound, ps_sound)
        ac.addOnCheckBoxChanged(checkbox_AutoRain, ps_AutoRain)
        ac.addOnCheckBoxChanged(checkbox_AutoWipers, ps_AutoWipers)
        ac.addOnCheckBoxChanged(checkbox_ManualRain, ps_ManualRain)
        ac.addOnCheckBoxChanged(checkbox_ManualWipers, ps_ManualWipers)

        ac.addOnValueChangeListener(spinnerSmokeAmount, ps_smoke_settings)
        ac.addOnValueChangeListener(spinnerSmokeThickness, ps_smoke_settings)
        ac.addOnValueChangeListener(spinnerRangeTrackLights, ps_range_tracklights)
        ac.addOnValueChangeListener(spinnerGlow, onspinnerGlow)
        ac.addOnValueChangeListener(spinnerRangeHeadLights, ps_range_headlights)
        ac.addOnCheckBoxChanged(checkbox_Tracklights, appTrackLightsToggle)
        ac.addOnClickedListener(btnFileTrack, runEditorTrack)
        ac.addOnClickedListener(btnFileTrack2, runEditorTrack2)
        ac.addOnClickedListener(btnFileCar, runEditorCar)
        ac.addOnClickedListener(btnFileCar2, runEditorCar2)

        createSpinner(ps_app, spinnerNightBright, "night__brightness_adjust",
                      int(night__brightness_adjust * 100),
                      10,10,100, 20, 0, gMaximumValues, 5, onspinnerNightBright, True)
        checkbox_Seasons = ac.addCheckBox(ps_app, "seasons")
        spinner_LIT_MULT = ac.addSpinner(ps_app, "")            # tracklights LIT_MULT spinner
        spinner_CAR_LIGHTS_LIT_MULT = ac.addSpinner(ps_app, "") # tracklights CAR_LIGHTS_LIT_MULT spinner
        spinner_SPECULAR_MULT = ac.addSpinner(ps_app, "")       # tracklights SPECULAR_MULT spinner
        lbNigh = ac.addLabel(ps_app, "night                    brightn. %")
        lbLIT_MULT = ac.addLabel(ps_app,            "per track                              LIT_MULT %")
        lbCAR_LIGHTS_LIT_MULT = ac.addLabel(ps_app, "per track                              CAR_LIT_MULT %")
        lbSPECULAR_MULT = ac.addLabel(ps_app,       "per track                              SPECULAR_MULT %")

        ac.setValue(checkbox_Seasons, gSeasonsActive)
        ac.setRange(spinner_LIT_MULT, 0, gMaximumValues)
        ac.setRange(spinner_CAR_LIGHTS_LIT_MULT, 0, gMaximumValues)
        ac.setRange(spinner_SPECULAR_MULT, 0, gMaximumValues)
        ac.setRange(spinnerNightBright, 0, gMaximumValues)
        ac.setStep(spinner_LIT_MULT, gSteps)
        ac.setStep(spinner_CAR_LIGHTS_LIT_MULT, gSteps)
        ac.setStep(spinner_SPECULAR_MULT, gSteps)
        ac.setValue(spinner_LIT_MULT, gLIT_MULT)
        ac.setValue(spinner_CAR_LIGHTS_LIT_MULT, gCAR_LIGHTS_LIT_MULT)
        ac.setValue(spinner_SPECULAR_MULT, gSPECULAR_MULT)
        ac.addOnValueChangeListener(spinner_LIT_MULT, ps_LIT_MULT)
        ac.addOnValueChangeListener(spinner_CAR_LIGHTS_LIT_MULT, ps_CAR_LIGHTS_LIT_MULT)
        ac.addOnValueChangeListener(spinner_SPECULAR_MULT, ps_SPECULAR_MULT)
        ac.addOnCheckBoxChanged(checkbox_Seasons, appSeasonToggle)


    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())
        ac.setText(lbChng, traceback.format_exc())

def appSetPositionsAndSizesPlan():
    global app, ps_app, spinner_ps_plan, checkbox_ps_plan_enable, lb_spinner_ps_plan, sPatchVersion
    global weathers, checkbox_ps_weather, lbDesc0, lbDesc1, lbDesc2
    global gAppsize, gHideBG, gAppShowOnStart, bPause
    global spinnerNightBright, lbNigh, lbCar, sLayout, checkbox_Tracklights, spinnerRangeTrackLights, gButtonH, gOrigHeadLightRange, gOrigSmokeAmount, spinnerSmokeThickness, spinnerSmokeAmount
    global spinnerGlow, spinner_LIT_MULT, spinner_CAR_LIGHTS_LIT_MULT, spinner_SPECULAR_MULT, lbGlow, lbLIT_MULT, lbCAR_LIGHTS_LIT_MULT, lbSPECULAR_MULT
    global buttonResetWeather, lbRang, gUseSymbols, lbChng, gUseScaling
    global bNewPlan, lbNewPlan, btnNewRandWeather, btnNewPlan, btnNewPlansave, lbNewPlanWeatherDuration, lbNewPlanRandom, edtFilename, chkBoxPlanLoop, btnPlanClose, btnToggleTransparencyPS
    global sNewWeatherClick, sNewWeather, btnFileCar, btnFileCar2, btnFileTrack, btnFileTrack2, sFileTrack2, sFileTrack, sFileCar, sFileCar2
    global spinnerTimeHolding, spinnerTimeChanging, spinnerTemperature_ambient, spinnerTemperature_road, spinnerWind_direction, spinnerWind_speed
    global chkBoxRandomTH, chkBoxRandomTC, chkBoxRandomAT, chkBoxRandomRT, chkBoxRandomWD, chkBoxRandomWS
    global checkbox_AutoRain, checkbox_AutoWipers, spinnerRain, spinnerWipers, spinnerSound, checkbox_ManualRain, checkbox_ManualWipers
    global gMaximumValues, gLoadSaveTracklightsRange, gLoadSaveTracklightsPerTrack, gShowSettings, btnSettingsToggle, checkbox_Seasons, gSeasonsActive, info
    global bSol2, bBase

    try:
        # ac.setTitle(ps_app, 'Sol Plan Selector')
        ac.setTitle(ps_app, '')
        ac.setIconPosition(ps_app, -20000,0)
        if gHideBG==True:
            ac.drawBorder(ps_app, 0)
            ac.setBackgroundOpacity(ps_app, 0)
        else:
            ac.drawBorder(ps_app, 1)
            ac.setBackgroundOpacity(ps_app, 0.5)

        dx=50
        dy=gButtonH/2
        if dy<18:
            dy=18
        fntsize=12
        mult=1.0
        if gAppsize>=0.1 and gAppsize<=4.0:
            mult=gAppsize
            dx=dx*mult
            dy=dy*mult
            fntsize=fntsize*mult

        # yposBase=dy*2+2
        yposBase=5
        xposBase=fntsize
        xpos=xposBase
        size_x = 365 * mult
        size_y = 550 * mult
        if gShowSettings==True:
            size_y += dy*4
            if gLoadSaveTracklightsRange==True:
                size_y += dy*4

        if bSol2 or bBase:
            if gShowSettings==True:
                  if gLoadSaveTracklightsRange==True:
                      ac.setSize(ps_app, size_x, size_y - dy*10)
                  else:
                      ac.setSize(ps_app, size_x, size_y - dy*15)
            else:
                ac.setSize(ps_app, size_x, size_y - dy*20)
        else:
            if bNewPlan==True:
                if gShowSettings==True:
                    if gLoadSaveTracklightsRange==True:
                        ac.setSize(ps_app, size_x*2.2, size_y+7*dy)
                    else:
                        ac.setSize(ps_app, size_x*2.2, size_y+2*dy     )
                else:
                    ac.setSize(ps_app, size_x*2.2, size_y - dy*5)
            else:
                if gShowSettings==True:
                    if gLoadSaveTracklightsRange==True:
                        ac.setSize(ps_app, size_x    , size_y+9*dy)
                    else:
                        ac.setSize(ps_app, size_x    , size_y+8*dy     )
                else:
                    ac.setSize(ps_app, size_x    , size_y - dy*5)

        ypos=yposBase
        ac.setPosition(lbDesc0, fntsize*3.5, ypos+fntsize/3) # "Enable dyn..."
        if not bSol2 and not bBase:
            ac.setPosition(checkbox_ps_plan_enable, fntsize+5, ypos+fntsize/2)
            ac.setPosition(lbWeatherinfo, fntsize, -fntsize*2)
            ac.setPosition(buttonResetWeather, size_x*0.7, ypos-fntsize/3)

            ac.setSize(checkbox_ps_plan_enable, size_x * 0.5, dy+2)
            ac.setSize(lbDesc0, size_x * 0.4, size_y)
            ac.setSize(buttonResetWeather, size_x*0.2, dy+1)
            ac.setFontSize(checkbox_ps_plan_enable, fntsize-1)
            ac.setFontSize(lbDesc0, fntsize*1.25)
            ac.setFontSize(buttonResetWeather, fntsize*0.85)
            ac.setFontSize(lbWeatherinfo, fntsize*1.5)

            ypos += dy*1.5
            ac.setPosition(spinner_ps_plan, size_x*0.6, ypos) # current plan and spinner
            ac.setPosition(lb_spinner_ps_plan, fntsize*3.5, ypos+fntsize*1.5)
            ac.setSize(spinner_ps_plan, size_x * 0.25, dy+2)
            ac.setSize(lb_spinner_ps_plan, size_x * 0.35, dy+1)
            ac.setFontSize(spinner_ps_plan, fntsize+1)
            ac.setFontSize(lb_spinner_ps_plan, fntsize+1)

            # newplan
            ac.setPosition(btnNewPlan           , size_x - size_x*0.075, ypos)
            ac.setSize(btnNewPlan               , size_x*0.05, dy+3)
        else:
            # ypos+=dy/2
            ac.setText(lbDesc0, "Sol2 implementation active, \nuse new 'Sol Planner' app to change weather")
            if bBase:
                ac.setText(lbDesc0, "Default CSP weather implementation active, \nno weather controls available")
            else:
                ac.setText(lbDesc0, "Sol2 implementation active, \nuse new 'Sol Planner' app to change weather")
            ac.setPosition(checkbox_ps_plan_enable, 20000, ypos+fntsize/2)
            ac.setPosition(lbWeatherinfo, 20000, -fntsize*2)
            ac.setPosition(buttonResetWeather, 20000, ypos-fntsize/3)
            ac.setPosition(spinner_ps_plan, 20000, ypos) # current plan and spinner
            ac.setPosition(lb_spinner_ps_plan, 20000, ypos+fntsize*1.5)
            ac.setPosition(btnNewPlan           , 20000, ypos)

        ac.setPosition(btnPlanClose         , size_x - size_x*0.075, ypos+dy*2)
        ac.setSize(btnPlanClose             , size_x*0.05, dy+3)

        # ac.setPosition(btnToggleTransparencyPS, -20000, ypos+dy*4)
        ac.setPosition(btnToggleTransparencyPS, size_x - size_x*0.075, ypos+dy*4)
        ac.setSize(btnToggleTransparencyPS    , size_x*0.05, dy+3)

        ac.setSize(btnNewPlansave           , size_x*0.2 , dy-2)
        # ac.setSize(lbNewPlan                , size_x*1.1, size_y*0.55) # place for the big plan
        if gShowSettings==True:
            ac.setSize(lbNewPlan            , size_x*1.1, size_y*0.675) # place for the big plan
        else:
            ac.setSize(lbNewPlan            , size_x*1.1, size_y*0.32) # place for the big plan

        ac.setSize(spinnerTimeHolding, size_x*0.3 , dy-2) # also used for "new plan's filename"
        ac.setSize(spinnerTimeChanging, size_x*0.3 , dy-2) # also used for "new plan's filename"
        ac.setSize(btnNewRandWeather        , size_x*0.2, fntsize+1) # rnd weather btn
        ac.setFontSize(btnNewRandWeather    , fntsize-1)

        ac.setSize(chkBoxPlanLoop        , size_x*0.5, fntsize) # rnd weather btn
        ac.setFontSize(chkBoxPlanLoop    , fntsize-1)

        ac.setSize(spinnerTemperature_ambient,size_x*0.3 , dy-4)
        ac.setSize(spinnerTemperature_road   ,size_x*0.3 , dy-4)
        ac.setSize(spinnerWind_direction     ,size_x*0.3 , dy-4)
        ac.setSize(spinnerWind_speed         ,size_x*0.3 , dy-4)
        ac.setSize(chkBoxRandomTH            ,size_x*0.3 , fntsize)
        ac.setSize(chkBoxRandomTC            ,size_x*0.3 , fntsize)
        ac.setSize(chkBoxRandomAT            ,size_x*0.3 , fntsize)
        ac.setSize(chkBoxRandomRT            ,size_x*0.3 , fntsize)
        ac.setSize(chkBoxRandomWD            ,size_x*0.3 , fntsize)
        ac.setSize(chkBoxRandomWS            ,size_x*0.3 , fntsize)

        ac.setSize(edtFilename, size_x*1.1 , dy-2)

        ac.setFontSize(btnNewPlan, fntsize)
        ac.setFontSize(btnPlanClose, fntsize)
        ac.setFontSize(btnToggleTransparencyPS, fntsize)
        ac.setFontSize(btnNewPlansave, fntsize-1)
        ac.setFontSize(lbNewPlan, fntsize)
        ac.setFontSize(lbNewPlanWeatherDuration  , fntsize+1)
        ac.setFontSize(lbNewPlanRandom  , fntsize+1)
        ac.setFontSize(spinnerTimeHolding , fntsize-1)
        ac.setFontSize(spinnerTimeChanging , fntsize-1)
        ac.setFontSize(spinnerTemperature_ambient, fntsize-1)
        ac.setFontSize(spinnerTemperature_road   , fntsize-1)
        ac.setFontSize(spinnerWind_direction     , fntsize-1)
        ac.setFontSize(spinnerWind_speed         , fntsize-1)
        ac.setFontSize(edtFilename, fntsize)

        ac.drawBorder(btnNewPlan, 0)
        ac.drawBorder(btnPlanClose, 0)
        ac.drawBorder(btnToggleTransparencyPS, 0)
        ac.drawBorder(lbNewPlan, 1)

        # ac.setBackgroundOpacity(edtFilename, 1)
        # ac.setFontAlignment(lbNewPlan, 'left')
        # ac.setBackgroundTexture(edtFilename, inputBgImage)
        # ac.drawBorder(edtFilename, drawEditorInputBorder if not forceDrawBorder else True)

        if bNewPlan==True:
            temperature_ambient = str(acinfo.physics.airTemp)
            temperature_road = str(acinfo.physics.roadTemp)
            wind_direction = str(round(acinfo.graphics.windDirection,1))
            wind_speed = str(random.randint(2, 15) if acinfo.graphics.windSpeed == 0 else round(acinfo.graphics.windSpeed,1))

            bPause=True
            ac.setValue(spinnerTemperature_road   , int(float(temperature_road)) )
            ac.setValue(spinnerTemperature_ambient, int(float(temperature_ambient)) )
            ac.setValue(spinnerWind_direction     , int(float(wind_direction)) )
            ac.setValue(spinnerWind_speed         , int(float(wind_speed)) )
            bPause=False

            ac.setText(btnNewPlan, '<')
            ac.setPosition(btnNewPlansave            , size_x*2 - size_x*0.1  , ypos - dy - fntsize-1)
            ac.setPosition(edtFilename               , size_x                 , ypos)
            ac.setPosition(spinnerTimeHolding        , size_x + size_x*0.5    , ypos + dy*2.1 + fntsize+1)
            ac.setPosition(spinnerTimeChanging       , size_x + size_x*0.5    , ypos + dy*3.1 + fntsize+1)
            ac.setPosition(spinnerTemperature_ambient, size_x + size_x*0.5    , ypos + dy*4.1 + fntsize+1)
            ac.setPosition(spinnerTemperature_road   , size_x + size_x*0.5    , ypos + dy*5.1 + fntsize+1)
            ac.setPosition(spinnerWind_direction     , size_x + size_x*0.5    , ypos + dy*6.1 + fntsize+1)
            ac.setPosition(spinnerWind_speed         , size_x + size_x*0.5    , ypos + dy*7.1 + fntsize+1)
            ac.setPosition(chkBoxRandomTH            , size_x + size_x*0.825  , ypos + dy*2.1 + fntsize+1)
            ac.setPosition(chkBoxRandomTC            , size_x + size_x*0.825  , ypos + dy*3.1 + fntsize+1)
            ac.setPosition(chkBoxRandomAT            , size_x + size_x*0.825  , ypos + dy*4.1 + fntsize+1)
            ac.setPosition(chkBoxRandomRT            , size_x + size_x*0.825  , ypos + dy*5.1 + fntsize+1)
            ac.setPosition(chkBoxRandomWD            , size_x + size_x*0.825  , ypos + dy*6.1 + fntsize+1)
            ac.setPosition(chkBoxRandomWS            , size_x + size_x*0.825  , ypos + dy*7.1 + fntsize+1)
            ac.setPosition(lbNewPlanWeatherDuration  , size_x                 , ypos - dy)
            ac.setPosition(lbNewPlanRandom           , size_x + size_x*0.875  , ypos - dy - fntsize/3)
            ac.setPosition(lbNewPlan                 , size_x                 , ypos + dy*10.5)
            ac.setPosition(btnNewRandWeather         , size_x*2 - size_x*0.1  , ypos + dy*9.5)
            ac.setPosition(chkBoxPlanLoop            , size_x                 , ypos + dy*9.5)
            ac.setText(lbNewPlan, sNewWeatherClick)
        else:
            ac.setText(btnNewPlan, '>')
            ac.setPosition(btnNewPlansave            , -20000, ypos)
            ac.setPosition(edtFilename               , -20000, ypos)
            ac.setPosition(spinnerTimeHolding        , -20000, ypos)
            ac.setPosition(spinnerTimeChanging       , -20000, ypos)
            ac.setPosition(spinnerTemperature_ambient, -20000, ypos)
            ac.setPosition(spinnerTemperature_road   , -20000, ypos)
            ac.setPosition(spinnerWind_direction     , -20000, ypos)
            ac.setPosition(spinnerWind_speed         , -20000, ypos)
            ac.setPosition(chkBoxRandomTH            , -20000, ypos)
            ac.setPosition(chkBoxRandomTC            , -20000, ypos)
            ac.setPosition(chkBoxRandomAT            , -20000, ypos)
            ac.setPosition(chkBoxRandomRT            , -20000, ypos)
            ac.setPosition(chkBoxRandomWD            , -20000, ypos)
            ac.setPosition(chkBoxRandomWS            , -20000, ypos)
            ac.setPosition(lbNewPlanWeatherDuration  , -20000, ypos)
            ac.setPosition(lbNewPlanRandom           , -20000, ypos)
            ac.setPosition(lbNewPlan                 , -20000, ypos)
            ac.setPosition(chkBoxPlanLoop            , -20000, ypos)
            ac.setPosition(btnNewRandWeather         , -20000, ypos)

        #  ypos += dy*3.5  # 7.2 no performance mode anymore
        # ac.setPosition(checkbox_PerformanceMode, xpos, ypos) # 'Performance mode (clear-24h-stars)'
        # ac.setSize(checkbox_PerformanceMode, fntsize*1.2, fntsize*1.2)

        if bSol2 or bBase:
            for weather in weathers:
                ac.setPosition(checkbox_ps_weather[weather], -20000, -20000)
        else:
            ypos += dy*3.7
            step = 0
            for weather in weathers:
                ac.setPosition(checkbox_ps_weather[weather], xpos, ypos + step)
                ac.setSize(checkbox_ps_weather[weather], fntsize*1.2, fntsize*1.2)
                ac.setFontSize(checkbox_ps_weather[weather], fntsize)
                if gUseScaling==True:
                    #ac.setFontSize(checkbox_ps_weather[weather], 0) # doesnt work for checkboxes
                    ac.setFontColor(checkbox_ps_weather[weather],1,1,1,0)      # doesnt work for some ppl
                if (weathers.index(weather) + 1) % 16 == 0:
                    xpos += 160 * mult
                    step = 0
                else:
                    step += dy

        if gUseScaling==True and not bSol2 and not bBase:
            ac.setPosition(lbDesc1,              fntsize*2 + 10, ypos-dy*0.3)
            ac.setPosition(lbDesc2, size_x*0.5 + fntsize/2 + 10, ypos-dy*0.3)
            ac.setSize(lbDesc1, size_x * 0.4, size_y)
            ac.setSize(lbDesc2, size_x * 0.4, size_y)
            ac.setFontSize(lbDesc1, fntsize*1.15)
            ac.setFontSize(lbDesc2, fntsize*1.15)
        else:
            ac.setPosition(lbDesc1, -20000, 0)
            ac.setPosition(lbDesc2, -20000, 0)

        xpos = xposBase
        if bSol2 or bBase:
            ypos = ypos + dy
        else:
            ypos = ypos + dy*11.5

        ac.setPosition(btnSettingsToggle, size_x - size_x*0.075, ypos+dy*5.5) # more options in plan window

        # ypos -= dy*4  # without headlightsrange slider
        # without active sound spinner:
        if bSol2 or bBase:
            ac.setPosition(checkbox_AutoRain, -20000, -20000)
            ac.setPosition(checkbox_AutoWipers, -20000, -20000)
            ac.setPosition(checkbox_ManualRain, -20000, -20000)
            ac.setPosition(checkbox_ManualWipers, -20000, -20000)
            ac.setPosition(spinnerRain, -20000, -20000)
            ac.setPosition(spinnerWipers, -20000, -20000)
            ac.setPosition(spinnerSound, -20000, -20000)
        else:
            ac.setPosition(checkbox_AutoRain, xpos, ypos+dy*4+fntsize*2) # autorain + autowipers checkboxes with headlights slider
            ac.setPosition(checkbox_AutoWipers, xpos+size_x * 0.5, ypos+dy*4+fntsize*2) # debug out
            ac.setPosition(checkbox_ManualRain, xpos, ypos+dy*5+fntsize*2.5) # autorain + autowipers checkboxes with headlights slider
            ac.setPosition(checkbox_ManualWipers, xpos+size_x * 0.5, ypos+dy*5+fntsize*2.5) # debug out
            ac.setPosition(spinnerRain, xpos+fntsize*2, ypos+dy*5+fntsize*2.25)
            ac.setPosition(spinnerWipers, xpos+fntsize+size_x * 0.55, ypos+dy*5+fntsize*2.25) # debug out v 5.3
            ac.setPosition(spinnerSound, -20000, 0) # hide sound spinner, not active atm

            # with active sound spinner:
            # ac.setPosition(checkbox_AutoWipers, xpos+size_x * 0.33, ypos+dy*4+fntsize*2)
            # ac.setPosition(checkbox_ManualRain, xpos, ypos+dy*5+fntsize*2.5) # autorain + autowipers checkboxes with headlights slider
            # ac.setPosition(checkbox_ManualWipers, xpos+size_x * 0.33, ypos+dy*5+fntsize*2.5)
            # ac.setPosition(spinnerRain, xpos+fntsize*2, ypos+dy*5+fntsize*2.25)
            # ac.setPosition(spinnerWipers, xpos+fntsize+size_x * 0.35, ypos+dy*5+fntsize*2.25) # debug out v 5.3
            # ac.setPosition(spinnerSound, xpos+fntsize+size_x * 0.66, ypos+dy*5+fntsize*2.25) # debug out 5.3

        ac.setSize(btnSettingsToggle    , size_x*0.05, dy+3)

        if bSol2 or bBase:
            ypos += dy*3.5
        else:
            ypos += dy*9

        if gShowSettings==True:
            if sFileTrack2!='':
                ac.setPosition(btnFileTrack           , xpos+(fntsize+2)-5, ypos)     # btnFileTrack calls editor
                ac.setPosition(btnFileTrack2          , xpos-(fntsize+2)-5, ypos)     # btnFileTrack calls editor
            else:
                ac.setPosition(btnFileTrack           , xpos+(fntsize+2)-15, ypos)     # btnFileTrack calls editor
                ac.setPosition(btnFileTrack2          , -20000, ypos)     # btnFileTrack calls editor

            # ypos += dy*2
            ac.setPosition(lbRang                     , xpos+fntsize*2, ypos-1)
            if gLoadSaveTracklightsRange==True:
                ## ac.setPosition(checkbox_Tracklights       , xpos          , ypos-1)
                ac.setPosition(checkbox_Tracklights       , xpos+(fntsize+2)*2.5          , ypos-1)
                ac.setPosition(lbRang                     , xpos+fntsize*3+5, ypos-1)

                ypos += fntsize*2
                ac.setPosition(spinnerRangeTrackLights    , xpos          , ypos-1)     # track lights range

                ypos += dy*1.7
                ac.setPosition(lbGlow                     , xpos + dx/2+2 , ypos)     # label
                ac.setPosition(spinnerGlow                , xpos          , ypos)     # track lights glow
            else:
                ac.setPosition(checkbox_Tracklights       , -20000        , ypos-1)
                ac.setPosition(spinnerRangeTrackLights    , -20000          , ypos-1)     # track lights range
                ac.setPosition(lbGlow                     , -20000 + dx/2+2 , ypos)     # label
                ac.setPosition(spinnerGlow                , -20000          , ypos)     # track lights glow

            if not bSol2 and not bBase:
                ypos += dy*2.1
                ac.setPosition(lbNigh                     , xpos + dx/2+2 , ypos+3)
                ac.setPosition(spinnerNightBright         , xpos          , ypos+1)   # night brightness
                ac.setPosition(checkbox_Seasons           , size_x*0.7    , ypos+1)
            else:
                ypos += dy
                ac.setPosition(lbNigh                     , -20000 , ypos+3)
                ac.setPosition(spinnerNightBright         , -20000 , ypos+1)   # night brightness
                ac.setPosition(checkbox_Seasons           , size_x*0.7, ypos+dy*0.75)

            ypos += dy*2.4
            ac.setPosition(spinner_LIT_MULT           , xpos          , ypos)       # LIT_MULT
            ac.setPosition(lbLIT_MULT                 , xpos + dx/2+2 , ypos+2)     # label
            ypos += dy*1.9
            ac.setPosition(spinner_CAR_LIGHTS_LIT_MULT, xpos          , ypos)       # CAR_LIGHTS_LIT_MULT
            ac.setPosition(lbCAR_LIGHTS_LIT_MULT      , xpos + dx/2+2 , ypos+2)     # label
            ypos += dy*1.9
            ac.setPosition(spinner_SPECULAR_MULT      , xpos          , ypos)       # SPECULAR_MULT
            ac.setPosition(lbSPECULAR_MULT            , xpos + dx/2+2 , ypos+2)     # label

            ypos += dy*2 # !!!!!!!!!!!!!!!!!! deactivated
            #ac.setPosition(lbCar, -20000, ypos+3+dy*2-fntsize)
            ac.setPosition(lbCar                  , xpos+dx/2+5, ypos)
            if sFileCar2!='':
                ac.setPosition(btnFileCar             , xpos+(fntsize+2)-5, ypos)
                ac.setPosition(btnFileCar2            , xpos-(fntsize+2)-5, ypos)
            else:
                ac.setPosition(btnFileCar             , xpos+(fntsize+2)-15, ypos)
                ac.setPosition(btnFileCar2            , -20000, ypos)

            ypos += dy
            ac.setPosition(spinnerRangeHeadLights     , xpos          , ypos)      # car headlights spinner
            # ac.setPosition(spinnerRangeHeadLights, -20000, ypos+1+dy*2+fntsize)  # NO car headlights spinner

            ypos += dy*2
            ac.setPosition(spinnerSmokeAmount        , xpos          , ypos)       # car headlights spinner
            ac.setPosition(spinnerSmokeThickness     , xpos          , ypos+dy*2)  #
        else:
            ac.setPosition(spinnerNightBright         , -20000, 1)   # night brightness
            ac.setPosition(checkbox_Seasons           , -20000, 1)   # night brightness
            ac.setPosition(lbNigh                     , -20000, 1)
            ac.setPosition(btnFileTrack               , -20000, 1)
            ac.setPosition(btnFileTrack2              , -20000, 1)
            ac.setPosition(btnFileCar                 , -20000, 1)
            ac.setPosition(btnFileCar2                , -20000, 1)
            ac.setPosition(lbRang                     , -20000, 1)
            ac.setPosition(checkbox_Tracklights       , -20000, 1)

            ac.setPosition(spinnerRangeTrackLights    , -20000, 1)     # track lights range
            ac.setPosition(lbGlow                     , -20000, 1)     # label
            ac.setPosition(spinnerGlow                , -20000, 1)     # track lights glow
            ac.setPosition(spinner_LIT_MULT           , -20000, 1)     # LIT_MULT
            ac.setPosition(lbLIT_MULT                 , -20000, 1)     # label
            ac.setPosition(spinner_CAR_LIGHTS_LIT_MULT, -20000, 1)     # CAR_LIGHTS_LIT_MULT
            ac.setPosition(lbCAR_LIGHTS_LIT_MULT      , -20000, 1)     # label
            ac.setPosition(spinner_SPECULAR_MULT      , -20000, 1)     # SPECULAR_MULT
            ac.setPosition(lbSPECULAR_MULT            , -20000, 1)     # label
            ac.setPosition(lbCar                      , -20000, 1)
            ac.setPosition(spinnerRangeHeadLights     , -20000, 1)     # car headlights spinner
            ac.setPosition(spinnerSmokeAmount         , -20000, 1)
            ac.setPosition(spinnerSmokeThickness      , -20000, 1)

        ac.setRange(spinnerRangeTrackLights, 0, gMaximumValues)
        ac.setRange(spinnerGlow, 0, gMaximumValues)
        ac.setRange(spinner_LIT_MULT, 0, gMaximumValues)
        ac.setRange(spinner_CAR_LIGHTS_LIT_MULT, 0, gMaximumValues)
        ac.setRange(spinner_SPECULAR_MULT, 0, gMaximumValues)
        ac.setRange(spinnerNightBright, 0, gMaximumValues)

        ac.setSize(btnFileTrack, fntsize+1, fntsize+2)
        ac.setSize(btnFileTrack2, fntsize+1, fntsize+2)
        ac.setSize(btnFileCar, fntsize+1, fntsize+2)
        ac.setSize(btnFileCar2, fntsize+1, fntsize+2)
        ac.setSize(lbRang, size_x * 0.4, size_y)
        ac.setSize(lbCar, size_x * 0.9, dy)
        ac.setSize(spinner_LIT_MULT, size_x * 0.9, dy * 1.25)
        ac.setSize(spinner_CAR_LIGHTS_LIT_MULT, size_x * 0.9, dy * 1.25)
        ac.setSize(spinner_SPECULAR_MULT, size_x * 0.9, dy * 1.25)
        ac.setSize(spinnerNightBright, size_x * 0.6, dy * 1.25)
        ac.setSize(checkbox_Seasons    , size_x * 0.4, dy * 1)
        ac.setSize(lbNigh, size_x * 0.9, dy)
        ac.setSize(lbGlow, size_x * 0.9, dy)
        ac.setSize(lbLIT_MULT, size_x * 0.9, dy)
        ac.setSize(lbCAR_LIGHTS_LIT_MULT, size_x * 0.9, dy)
        ac.setSize(lbSPECULAR_MULT, size_x * 0.9, dy)
        ac.setSize(spinnerGlow, size_x * 0.9, dy * 1.25)
        ac.setSize(spinnerRangeTrackLights, size_x * 0.9, dy * 1.25)
        ac.setSize(spinnerRangeHeadLights, size_x * 0.9, dy * 1.25)
        ac.setSize(spinnerSmokeAmount, size_x * 0.9, dy * 1.25)
        ac.setSize(spinnerSmokeThickness, size_x * 0.9, dy * 1.25)
        ac.setSize(spinnerRain, size_x * 0.25, dy * 1.25)
        ac.setSize(spinnerWipers, size_x * 0.25, dy * 1.25)
        ac.setSize(spinnerSound, size_x * 0.25, dy * 1.25)
        ac.setSize(checkbox_Tracklights, size_x * 0.9, dy * 1)
        ac.setSize(checkbox_AutoRain, size_x * 0.25, dy * 1)
        ac.setSize(checkbox_AutoWipers, size_x * 0.25, dy * 1)
        ac.setSize(checkbox_ManualRain, size_x * 0.25, dy * 1)
        ac.setSize(checkbox_ManualWipers, size_x * 0.25, dy * 1)

        ac.setFontSize(checkbox_Tracklights, fntsize-1)
        ac.setFontSize(chkBoxPlanLoop, fntsize-1)
        ac.setFontSize(lbRang, fntsize*1)
        ac.setFontSize(spinnerNightBright, fntsize+1)
        ac.setFontSize(spinnerGlow, fntsize+1)
        ac.setFontSize(spinnerRangeTrackLights, fntsize+1)
        ac.setFontSize(spinner_LIT_MULT, fntsize+1)
        ac.setFontSize(spinner_CAR_LIGHTS_LIT_MULT, fntsize+1)
        ac.setFontSize(spinner_SPECULAR_MULT, fntsize+1)
        ac.setFontSize(spinnerRangeHeadLights, fntsize+1)
        ac.setFontSize(spinnerSmokeAmount, fntsize+1)
        ac.setFontSize(spinnerSmokeThickness, fntsize+1)
        ac.setFontSize(lbNigh, fntsize+1)
        ac.setFontSize(lbGlow, fntsize+1)
        ac.setFontSize(lbLIT_MULT, fntsize+1)
        ac.setFontSize(lbCAR_LIGHTS_LIT_MULT, fntsize+1)
        ac.setFontSize(lbSPECULAR_MULT, fntsize+1)
        ac.setFontSize(lbCar, fntsize*1.2)

        ac.setFontSize(btnFileTrack, fntsize-2)
        ac.setFontSize(btnFileTrack2, fntsize-2)
        ac.setFontSize(btnFileCar, fntsize-2)
        ac.setFontSize(btnFileCar2, fntsize-2)
        ac.setFontSize(checkbox_Seasons, fntsize+1)
        ac.setFontSize(checkbox_AutoRain, fntsize+1)
        ac.setFontSize(checkbox_AutoWipers, fntsize+1)
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())
        ac.setText(lbChng, traceback.format_exc())

##############################################################################

def setHotKeys():
    global hotKeyListener
    try:
        hotKeyListener = threading.Thread(target=listen_key)
        hotKeyListener.daemon = True
        hotKeyListener.start()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

def listen_key():
    global gModifier, hotKeyToggleMain, hotKeyTogglePlan, hotKeyPlusBig, hotKeyMinusBig, hotKeyPlusSmall, hotKeyMinusSmall, hotKeyPlusTiny, hotKeyMinusTiny
    try:
        user32 = ctypes.windll.user32
        lMod = 0
        if 'ALT' in gModifier:
            lMod = lMod | win32con.MOD_ALT
        if 'CTRL' in gModifier:
            lMod = lMod | win32con.MOD_CONTROL
        if 'SHIFT' in gModifier:
            lMod = lMod | win32con.MOD_SHIFT
        if 'WIN' in gModifier:
            lMod = lMod | win32con.MOD_WIN
        HOTKEYS = {   # use modifiers
            1: (ord(hotKeyToggleMain), lMod),
            2: (ord(hotKeyTogglePlan), lMod),
            3: (ord(hotKeyPlusBig), lMod),
            4: (ord(hotKeyMinusBig), lMod),
            5: (ord(hotKeyPlusSmall), lMod),
            6: (ord(hotKeyMinusSmall), lMod),
            7: (ord(hotKeyPlusTiny), lMod),
            8: (ord(hotKeyMinusTiny), lMod)
        }

        def handle_1():
            appShowMain()
        def handle_2():
            appShowPlan()
        def handle_3():
            timeOffsetPlusBig()
        def handle_4():
            timeOffsetMinusBig()
        def handle_5():
            timeOffsetPlusSmall()
        def handle_6():
            timeOffsetMinusSmall()
        def handle_7():
            timeOffsetPlusSmaller()
        def handle_8():
            timeOffsetMinusSmaller()

        HOTKEY_ACTIONS = {
            1: handle_1,
            2: handle_2,
            3: handle_3,
            4: handle_4,
            5: handle_5,
            6: handle_6,
            7: handle_7,
            8: handle_8
        }

        for id, (vk, modifiers) in HOTKEYS.items():
            if not user32.RegisterHotKey(None, id, modifiers, vk):
                # ac.console("Hotkey %d NOT Registered: %s" % (id, HOTKEYS.items()) )
                # ac.log("sol_weather app: Hotkey %d NOT Registered: %s" % (id, HOTKEYS.items()) )
                ac.console("sol_weather app: Hotkey NOT Registered: " + str(gModifier) + ' ' + chr(vk))
                ac.log("sol_weather app: could not register Hotkey: " + str(gModifier) + ' ' + chr(vk))
        try:
            byref = ctypes.byref
            msg = ctypes.wintypes.MSG()
            while user32.GetMessageA(byref(msg), None, 0, 0) != 0:
                if msg.message == win32con.WM_HOTKEY:
                    action_to_take = HOTKEY_ACTIONS.get(msg.wParam)
                    if action_to_take:
                        action_to_take()
                user32.TranslateMessage(byref(msg))
                user32.DispatchMessageA(byref(msg))
                # time.sleep(0.001)
                # time.sleep(0.05)
                time.sleep(0.01)
        finally:
            for id in HOTKEYS.keys():
                user32.UnregisterHotKey(None, id)
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

##############################################################################

def ps_newplan(*args):
    global ps_app, app
    global bNewPlan, lbNewPlan, btnNewPlan, btnNewPlansave, lbNewPlanWeatherDuration, spinnerTimeHolding, spinnerTimeChanging
    bNewPlan=not bNewPlan
    ac.setText(lbNewPlan, '')
    appSetPositionsAndSizesPlan()

def ps_newplan_save(*args):
    global lb_spinner_ps_plan, checkbox_ps_weather, ps_app, sol_weather_plan, plans_path, plans
    global bNewPlan, lbNewPlan, btnNewPlan, btnNewPlansave, lbNewPlanWeatherDuration
    global gPlanLoop, checkbox_ps_plan_enable, spinner_ps_plan, bPause

    bOK = False
    # uncheck all
    for k, v in checkbox_ps_weather.items():
        ac.setValue(v, 0)

    # read current conditions
    temperature_ambient = str(acinfo.physics.airTemp)
    temperature_road    = str(acinfo.physics.roadTemp)
    wind_direction      = str(round(acinfo.graphics.windDirection,1))
    wind_speed          = str(random.randint(2, 15) if acinfo.graphics.windSpeed == 0 else round(acinfo.graphics.windSpeed,1))
    try:
        s = ac.getText(lbNewPlan)
        if s!=sNewWeatherClick: # something was added
            ss = s.split('\n')
            new_filedata = ''
            wNo = len(ss)
            idx = -1
            # build weather plan from selected
            for sss in ss:
                idx+=1
                w = sss.replace(' ','').split(',')
                if len(w)>8:
                    bOK = True
                    name = w[0]
                    name = name.replace(' ','').encode('ascii', 'ignore').decode('ascii').strip()  # remove weather symbols
                    #(timeholding) (timechanging) (temperature_ambient) (temperature_road) (wind_direction) (wind_speed)
                    # (gRandomTH) (gRandomTC) (gRandomAT) (gRandomRT) (gRandomWD) (gRandomWS)
                    timeholding         = float(w[1]) * 60
                    timechanging        = float(w[2]) * 60
                    temperature_ambient = float(w[3])
                    temperature_road    = float(w[4])
                    wind_direction      = float(w[5])
                    wind_speed          = float(w[6])
                    sRandomTH           =  True if str(w[7])=='True' else False
                    sRandomTC           =  True if str(w[8])=='True' else False
                    sRandomAT           =  True if str(w[9])=='True' else False
                    sRandomRT           =  True if str(w[10])=='True' else False
                    sRandomWD           =  True if str(w[11])=='True' else False
                    sRandomWS           =  True if str(w[12])=='True' else False
                    new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({\n    weather = "' + name + '",'
                    if sRandomTH==True:
                        new_filedata +=  '\n    time_holding  = random(' + str(round(timeholding-timeholding/2,1)) + ',' + str(round(timeholding+timeholding/2,1)) + ')'
                    else:
                        new_filedata +=  '\n    time_holding  = ' + str(round(timeholding,1))

                    if idx+1==wNo and gPlanLoop==False:
                        new_filedata += ',\n    time_changing = -1'
                    else:
                        if sRandomTC==True:
                            new_filedata += ',\n    time_changing = random(' + str(round(timechanging-timechanging/2,1)) + ',' + str(round(timechanging+timechanging/2,1)) + ')'
                        else:
                            new_filedata += ',\n    time_changing = ' + str(round(timechanging,1))
                    if sRandomAT==True:
                        new_filedata += ',\n    temperature_ambient = random(' + str(round(max(0,temperature_ambient-5),1)) + ', ' + str(round(temperature_ambient+5,1)) + ')'
                    else:
                        new_filedata += ',\n    temperature_ambient = ' + str(round(temperature_ambient,1))
                    if sRandomRT==True:
                        new_filedata += ',\n    temperature_road = random(' + str(round(max(0,temperature_road-5),1)) + ', ' + str(round(temperature_road+5,1)) + ')'
                    else:
                        new_filedata += ',\n    temperature_road = ' + str(round(temperature_road,1))
                    if sRandomWD==True:
                        new_filedata += ',\n    wind_direction = random(' + str(round(max(0,wind_direction-180),0)) + ', ' + str(round(wind_direction+180,0)) + ')'
                    else:
                        new_filedata += ',\n    wind_direction = ' + str(round(wind_direction,0))
                    if sRandomWS==True:
                        new_filedata += ',\n    wind_speed = random(' + str(round(0,1)) + ',' + str(round(wind_speed*2,1)) + ')\n})\n'
                    else:
                        new_filedata += ',\n    wind_speed = ' + str(round(wind_speed,1)) + '\n})\n'

            if bOK==True:
                ac.ext_pauseFsWatching() #####
                fname = ac.getText(edtFilename) + '.lua'
                fname = fname.replace('.lua.lua', '.lua')

                # create new plan file in "assettocorsa\extension\weather-controllers\sol\weather_plans\"
                if os.path.isdir(plans_path):
                    with codecs.open(plans_path + fname, 'w', 'utf_8', errors='ignore') as file:
                        file.write(new_filedata)

                    # update "SOL__WEATHER_PLAN.lua" to use new created plan file
                    filedata = ''
                    with codecs.open(sol_weather_plan, 'r', 'utf_8', errors='ignore') as file:
                        filedata = file.read()
                    filedata = re.sub(r"--.*", "", filedata)  # delete comments
                    new_filedata = re.search(r"use_dynamic_weather_plan.*", filedata).group(0) + "\n"
                    new_filedata += re.search(r"__sol_weatherplans__path.*", filedata).group(0) + "\n"
                    # new_filedata += 'dofile (__sol_weatherplans__path.."' + fname + r'")'
                    # new_filedata += 'dofile (__sol_ctrl__path.."weather_plans/' + fname + '")'
                    new_filedata += 'dofile (__sol_weatherplans__path.."' + fname + '")'
                    with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as file:
                        file.write(new_filedata)

                planSelectorAddPlans()
                bPause=True
                ac.setValue(spinner_ps_plan, plans.index(fname) )
                bPause=False
                ac.setText(lb_spinner_ps_plan, fname )
                ps_plan_enable(fname,1)

                #weather_orig = ''
                # ac.setValue(checkbox_ps_plan_enable, 1)
                # appSetPlan(ac.getText(lb_spinner_ps_plan))

                ac.ext_resumeFsWatching() #####
                appApplyTrackConfig()

                # update lights, as they would be 100% again

        # hide weatheradd stuff
        bNewPlan=False
        appSetPositionsAndSizesPlan()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

##############################################################################

def planSelectorAddPlans():
    global spinner_ps_plan, checkbox_ps_plan_enable, sol_weather_plan, plans_path, plans, lb_spinner_ps_plan, bPause
    global onlinesession, lbDesc0, bDynamicWeather, weather_orig, lbWeatherinfo
    global last_weather
    # global bUpdateNeeded, updateID, updateTimer, last_weather
    if onlinesession==True or bPause==True:
        return
    try:
        filedata = ''
        if os.path.isfile(sol_weather_plan):
            with codecs.open(sol_weather_plan, 'r', 'utf_8', errors='ignore') as file:
                filedata = file.read()
            filedata = re.sub(r"--.*", "", filedata) # delete comments
        bLocalDynamicWeather = False
        stmp = appReadCFGValue(sol_weather_plan, '', 'use_dynamic_weather_plan','false')
        if stmp.lower()=='true':
            bLocalDynamicWeather = True

        bPause=True
        plans = []
        if os.path.isdir(plans_path):
            for f in os.listdir(plans_path):
                if os.path.isfile(os.path.join(plans_path, f)):
                    plans.append(f)
            if len(plans) != 0:
                if spinner_ps_plan>0:
                    ac.setText(spinner_ps_plan, "plans found: " + str(len(plans))) # plan spinner
                    ac.setRange(spinner_ps_plan, 0, len(plans) - 1)
                current_plan = re.search(r"dofile.*", filedata)
                if current_plan and bLocalDynamicWeather:
                    current_plan = current_plan.group(0)
                    current_plan = re.search(r"\"(.*)\"", current_plan)
                    if checkbox_ps_plan_enable>0:
                        ac.setValue(checkbox_ps_plan_enable, 1)
                        ac.setText(lb_spinner_ps_plan, current_plan.group(1))
                        ac.setFontColor(lb_spinner_ps_plan,0.25,1,0.25,1)
                        ac.setValue(spinner_ps_plan, plans.index(current_plan.group(1)))
                    # ac.setValue(spinner_ps_plan, 1)
                else:
                    if checkbox_ps_plan_enable>0:
                        ac.setValue(checkbox_ps_plan_enable, 0)
                        ac.setText(lb_spinner_ps_plan, plans[0])
                        ac.setFontColor(lb_spinner_ps_plan,0.5,0.5,0.5,0.5)
                    if 'add_weather_slot' in filedata:
                        # found one single weather in "extension\weather-controllers\sol\SOL__WEATHER_PLAN.lua"
                        wname=''
                        for line in filedata.split('\n'):
                            if 'weather =' in line or 'weather=' in line:
                                spl=line.split('=')
                                if len(spl)>1:
                                    wname=str( spl[1] ).replace('"','').replace(',','').strip()
                        for k, v in checkbox_ps_weather.items(): # clear all other checkboxes
                            if k == wname and 'use_dynamic_weather_plan=true' in filedata.replace(' ',''):
                                ac.setValue(v, 1)
                                wname = wname.replace(' ','').encode('ascii', 'ignore').decode('ascii').strip()  # remove weather symbols
                                last_weather=wname
                                # bUpdateNeeded=True
                                # updateID=v
                                # updateTimer=0.0
                            else:
                                ac.setValue(v, 0)
        if lbWeatherinfo>0:
            if not bLocalDynamicWeather: # and not gKeepPlanSettings:
                if weather_orig!='':
                    if onlinesession:
                        ac.setText(lbWeatherinfo, 'Online weather: "' + weather_orig + '"')
                    else:
                        ac.setText(lbWeatherinfo, 'CM weather: "' + weather_orig + '"')
            else:
                ac.setText(lbWeatherinfo, '')

        bPause=False
        if lbDesc0>0:
            ac.setText(lbDesc0, 'Enable dynamic weather\n-use a plan ('+str(len(plans))+'):\n\n-or force a weather:')
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

def ps_plan_enable(name, value):
    global bPause, sol_weather_plan, spinnerRangeTrackLights, gLightsON, lb_spinner_ps_plan, lbWeatherinfo, weather_orig
    global bPause, onlinesession, bDynamicWeather, bForcedWeather
    if onlinesession==True or bPause==True:
        return
    try:
        bPause = True
        ac.ext_pauseFsWatching() #####
        for k, v in checkbox_ps_weather.items():
            ac.setValue(v, 0)

        filedata = ''
        with codecs.open(sol_weather_plan, 'r', 'utf_8', errors='ignore') as file:
            filedata = file.read()

        if value == 1:
            bForcedWeather  = False
            bDynamicWeather = True
            filedata = re.sub(r"use_dynamic_weather_plan.*", "use_dynamic_weather_plan = true", filedata)
            with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as file:
                file.write(filedata)
            ac.setFontColor(lb_spinner_ps_plan,0.25,1,0.25,1)
            appSetPlan(ac.getText(lb_spinner_ps_plan))
        else:
            bDynamicWeather = False
            #with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as f: # write
            #    f.write('use_dynamic_weather_plan = false\n')
            filedata = re.sub(r"use_dynamic_weather_plan.*", "use_dynamic_weather_plan = false", filedata)
            with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as file:
                file.write(filedata)
            ac.setValue(checkbox_ps_plan_enable,0)
            ac.setFontColor(lb_spinner_ps_plan,0.5,0.5,0.5,0.5)

        if not bDynamicWeather: # and not gKeepPlanSettings:
            if weather_orig!='':
                if onlinesession:
                    ac.setText(lbWeatherinfo, 'Online weather: "' + weather_orig + '"')
                else:
                    ac.setText(lbWeatherinfo, 'CM weather: "' + weather_orig + '"')
        else:
            ac.setText(lbWeatherinfo, '')

        bPause = False
        # ps_reset_weather()
        ac.ext_resumeFsWatching() #####
        # update lights, as they would be 100% again
        appApplyTrackConfig()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

def ps_set_plan(*args):
    global lb_spinner_ps_plan, spinner_ps_plan, plans
    global bPause
    if bPause==True:
        return
    ac.setValue(checkbox_ps_plan_enable, 1)
    ac.setText(lb_spinner_ps_plan, plans[(int(ac.getValue(spinner_ps_plan)))] )
    appSetPlan(ac.getText(lb_spinner_ps_plan))

def appSetPlan(planfilename):
    global spinner_ps_plan, sol_weather_plan, plans, lb_spinner_ps_plan, checkbox_ps_weather, lbWeatherinfo
    global spinnerRangeTrackLights, gLightsON, checkbox_ps_plan_enable, plans_path
    try:
        ac.ext_pauseFsWatching() #####
        filedata = ''
        with codecs.open(sol_weather_plan, 'r', 'utf_8', errors='ignore') as file:
            filedata = file.read()
        filedata = re.sub(r"--.*", "", filedata)  # delete comments
        # new_filedata = re.search(r"use_dynamic_weather_plan.*", filedata).group(0) + "\n"
        # new_filedata += re.search(r"__sol_weatherplans__path.*", filedata).group(0) + "\n"

        new_filedata = 'use_dynamic_weather_plan = true\n'
        new_filedata += '__sol_weatherplans__path = "' + plans_path + '"\n'

        # new_filedata += 'dofile(__sol_weatherplans__path.."' + planfilename + r'")'
        # new_filedata += 'dofile (__sol_ctrl__path.."weather_plans/' + planfilename + '")\n'
        new_filedata += 'dofile (__sol_weatherplans__path.."' + planfilename + '")'
        ac.setFontColor(lb_spinner_ps_plan,0.25,1,0.25,1)
        bPause=True
        for k, v in checkbox_ps_weather.items():
            ac.setValue(v, 0)
        ac.setValue(checkbox_ps_plan_enable, 1)
        with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as file:
            file.write(new_filedata)
        # ps_reset_weather()
        bPause=False
        ac.ext_resumeFsWatching() #####
        # update lights, as they would be 100% again
        appApplyTrackConfig()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

def ps_force_weather(name, value):
    global sol_weather_plan, lbChng, checkbox_ps_weather, lb_spinner_ps_plan, spinnerRangeTrackLights
    global gLightsON, checkbox_ps_plan_enable
    global bNewPlan, lbNewPlan, spinnerTimeHolding, spinnerTimeChanging, edtFilename, sNewWeatherClick, gRandomTH, gRandomTC, gRandomAT, gRandomRT, gRandomWD, gRandomWS
    global spinnerRain, spinnerWipers, gAutoWipers, gAutoRain, lbWeatherinfo, weather_orig
    global bPause, last_plan, onlinesession, bDynamicWeather, bForcedWeather, last_weather, plans_path
    global sVarTemps, sVarWind, bAddTransition, lastfiledata, bRemoveAfterTransitionTime, iTransitiontime
    if onlinesession==True or bPause==True:
        if value==0 and weather_orig!='':
            ac.setText(lbWeatherinfo, 'Online weather: "' + weather_orig + '"')
        return

    try:
        if bNewPlan==True:
            # create new plan selector mode
            s = ac.getText(lbNewPlan)
            s = s.encode('ascii', 'ignore').decode('ascii').strip() # remove weather symbols

            timeholding         = ac.getValue(spinnerTimeHolding)
            timechanging        = ac.getValue(spinnerTimeChanging)
            temperature_ambient = ac.getValue(spinnerTemperature_ambient)
            temperature_road    = ac.getValue(spinnerTemperature_road)
            wind_direction      = ac.getValue(spinnerWind_direction)
            wind_speed          = ac.getValue(spinnerWind_speed)
            if s==sNewWeatherClick:
                s =            name + ', ' + str(timeholding) + ', ' + str(timechanging) + ', ' + str(temperature_ambient) + ', ' + str(temperature_road) + ', ' + str(wind_direction) + ', ' + str(wind_speed) + ', ' + str(gRandomTH) + ', ' + str(gRandomTC)+ ', ' + str(gRandomAT) + ', ' + str(gRandomRT) + ', ' + str(gRandomWD) + ', ' + str(gRandomWS)
            else:
                s = s + '\n' + name + ', ' + str(timeholding) + ', ' + str(timechanging) + ', ' + str(temperature_ambient) + ', ' + str(temperature_road) + ', ' + str(wind_direction) + ', ' + str(wind_speed) + ', ' + str(gRandomTH) + ', ' + str(gRandomTC)+ ', ' + str(gRandomAT) + ', ' + str(gRandomRT) + ', ' + str(gRandomWD) + ', ' + str(gRandomWS)
            ac.setText(lbNewPlan, s)
            # build basic filename
            fname = ''
            ss = s.split('\n')
            idx = -1
            bOK = False
            for sss in ss:
                idx+=1
                w = sss.split(',')
                if len(w)>1:
                    bOK = True
                    name = w[0]
                    name = name.encode('ascii', 'ignore').decode('ascii').strip()  # remove weather symbols
                    if idx<2:
                        fname = fname + name
            if bOK==True:
                ac.setText(edtFilename,fname+'.lua')
        else:
            name = name.encode('ascii', 'ignore').decode('ascii').strip() # remove weather symbols
            bPause=True
            for k, v in checkbox_ps_weather.items(): # clear all other checkboxes
                if k != name:
                    ac.setValue(v, 0)
            bPause=False

            if value==1:
                # ac.ext_pauseFsWatching() #####
                bDynamicWeather=False # for ui display only, its a plan after all
                bForcedWeather=True
                bPause=True
                ac.setValue(checkbox_ps_plan_enable, 0)
                ac.setText(lbWeatherinfo, '')
                ac.setFontColor(lb_spinner_ps_plan,0.5,0.5,0.5,0.5)
                bPause=False

                temperature_ambient = str(acinfo.physics.airTemp) # get current conditions
                temperature_road    = str(acinfo.physics.roadTemp)
                wind_direction      = str(acinfo.graphics.windDirection)
                wind_speed          = str(acinfo.graphics.windSpeed)
                if last_weather!='' and last_weather in weathers:
                    temperature_ambient = str( sVarTemps[weathers.index(last_weather)] )
                    temperature_road    = str( sVarTemps[weathers.index(last_weather)]+5 )
                    wind_direction      = str(acinfo.graphics.windDirection)
                    wind_speed          = str( sVarWind[weathers.index(last_weather)] )
                    # wind_speed          = str(random.randint(2, 15) if acinfo.graphics.windSpeed == 0 else round(acinfo.graphics.windSpeed,1))

                new_temperature_ambient = str( sVarTemps[weathers.index(name)] )
                new_temperature_road    = str( sVarTemps[weathers.index(name)]+5 )
                new_wind_direction      = str(acinfo.graphics.windDirection)
                new_wind_speed          = str( sVarWind[weathers.index(name)] )
                #ac.log(name + '   ' + last_weather + '   ' + weathers[weathers.index(name)] + '\n'
                #            + str(weathers.index(name)) + ' '
                #            + str(sVarWind[weathers.index(name)])  + '   ' + str( sVarTemps[weathers.index(name)] ) )
                try:
                    filedata = ''
                    if os.path.isfile(sol_weather_plan):
                        with codecs.open(sol_weather_plan, 'r', 'utf_8', errors='ignore') as f:
                            filedata = f.read()
                        filedata = re.sub(r"--.*", "", filedata) # delete comments
                    new_filedata = 'use_dynamic_weather_plan = true\n'
                    # new_filedata += re.search(r"__sol_weatherplans__path.*", filedata).group(0) + "\n"

                    # original plan file:
                    # use_dynamic_weather_plan = true'
                    # __sol_weatherplans__path = "extension\\weather-controllers\\sol\\weather_plans\\"
                    new_filedata += '__sol_weatherplans__path = "' + plans_path + '"'


                    lastfiledata = 'use_dynamic_weather_plan = true\n'
                    # lastfiledata += re.search(r"__sol_weatherplans__path.*", filedata).group(0) + "\n"#
                    lastfiledata += '__sol_weatherplans__path = "' + plans_path + '"'
                    lastfiledata += '\n__SOL_WEATHER_PLAN:add_weather_slot({\n    weather = "' + name + '"'
                    lastfiledata += ',\n    time_holding  = 1000000,\n    time_changing = -1'
                    lastfiledata += ',\n    temperature_ambient = ' + str(new_temperature_ambient) + ',\n    temperature_road = ' + str(new_temperature_road)
                    lastfiledata += ',\n    wind_direction = ' + str(new_wind_direction) + ',\n    wind_speed = ' + str(new_wind_speed)
                    lastfiledata += '\n})'

                    if bAddTransition:
                        new_filedata += '\n__SOL_WEATHER_PLAN:add_weather_slot({\n    weather = "' + last_weather + '"'
                        new_filedata += ',\n    time_holding  = 0,\n    time_changing = ' + str(iTransitiontime)
                        new_filedata += ',\n    temperature_ambient = ' + str(temperature_ambient) + ',\n    temperature_road = ' + str(temperature_road)
                        new_filedata += ',\n    wind_direction = ' + str(wind_direction) + ',\n    wind_speed = ' + str(wind_speed)
                        new_filedata += '\n})\n'
                        # now also start thread to remove old weather after 5.2 seconds
                        if bRemoveAfterTransitionTime:
                            threadWait5Secs()

                    new_filedata += '\n__SOL_WEATHER_PLAN:add_weather_slot({\n    weather = "' + name + '"'
                    new_filedata += ',\n    time_holding  = 1000000,\n    time_changing = -1'
                    new_filedata += ',\n    temperature_ambient = ' + str(new_temperature_ambient) + ',\n    temperature_road = ' + str(new_temperature_road)
                    new_filedata += ',\n    wind_direction = ' + str(new_wind_direction) + ',\n    wind_speed = ' + str(new_wind_speed)
                    new_filedata += '\n})'
                    #ac.log(name + '   ' + last_weather + '   ' + weathers[weathers.index(name)] + '\n'
                    #            + str(weathers.index(name)) + ' '
                    #            + str(new_wind_speed)  + '   ' + str( new_temperature_ambient ) )
                    with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as f: # write
                        f.write(new_filedata)
                except:
                    ac.log("SolWeatherApp error writing " + sol_weather_plan + '\n' + traceback.format_exc())

                last_weather = name
                # ps_reset_weather()
                # ac.ext_resumeFsWatching() #####
                # update lights, as they would be 100% again
                appApplyTrackConfig()
            else:
                bPause=True
                ac.ext_pauseFsWatching() #####
                ac.setValue(checkbox_ps_plan_enable, 0)
                bPause=False
                bDynamicWeather=False
                bForcedWeather=False
                appWriteCFGValue(sol_weather_plan, '', 'use_dynamic_weather_plan', 'false', True)
                #with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as f: # write
                #    f.write('use_dynamic_weather_plan = false\n')

                # ps_reset_weather()
                ac.ext_resumeFsWatching() #####
                if weather_orig!='':
                    ac.setText(lbWeatherinfo, 'CM weather: "' + weather_orig + '"')

                # update lights, as they would be 100% again
                appApplyTrackConfig()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

##############################################################################

def return_accel(temp):
    global accel_offset, lbChng, lastspeed, speed
    time.sleep(3)
    ac.setText(lbChng, '')
    accel_offset = temp
    # speed = lastspeed

def set_offset(offset):
    global accel_offset, thread, lbChng, lastspeed, speed
    global bPatchActive
    try:
        ac.setText(lbChng, '---   waiting for time to be set   ---')
        temp = accel_offset
        accel_offset = 0
        # lastspeed = speed
        speed = 0
        if bPatchActive==True:
            ac.ext_weatherTimeOffset(offset)
            if not thread or not thread.is_alive():
                thread = threading.Thread(target=return_accel, args=(temp,))
                thread.start()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

def set_offset_simple(offset):
    try:
        ac.ext_weatherTimeOffset(offset)
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

def timeOffsetPlusSmaller(*args):
    global gOffsetSmaller
    set_offset(gOffsetSmaller)
def timeOffsetMinusSmaller(*args):
    global gOffsetSmaller
    set_offset(-gOffsetSmaller)

def timeOffsetPlusSmall(*args):
    global gOffsetSmall
    set_offset(gOffsetSmall)
def timeOffsetMinusSmall(*args):
    global gOffsetSmall
    set_offset(-gOffsetSmall)

def timeOffsetPlusBig(*args):
    global gOffsetBig
    set_offset(gOffsetBig)
def timeOffsetMinusBig(*args):
    global gOffsetBig
    set_offset(-gOffsetBig)

def timeOffsetMinusDay(*args):
    set_offset_simple(-24 * 60 * 60)
def timeOffsetPlusDay(*args):
    set_offset_simple(24 * 60 * 60)
def timeOffsetMinusMonth(*args):
    set_offset_simple(-28 * 24 * 60 * 60)
def timeOffsetPlusMonth(*args):
    set_offset_simple(28 * 24 * 60 * 60)
def timeOffsetMinusYear(*args):
    set_offset_simple(-365 * 24 * 60 * 60)
def timeOffsetPlusYear(*args):
    set_offset_simple(365 * 24 * 60 * 60)

def timeOffsetMinusPlaySmall(*args):
    global speed
    speed -= 1
    if speed<-200:
        speed = -200
def timeOffsetPlusPlaySmall(*args):
    global speed
    speed += 1
    if speed>200:
        speed = 200
def timeOffsetMinusPlay(*args):
    global speed
    speed -= 7
    if speed<-200:
        speed = -200
def timeOffsetPlusPlay(*args):
    global speed
    speed += 7
    if speed>200:
        speed = 200
def timeAccelDown(*args):
    global accel, accel_offset
    accel -= 1
    accel_offset = accel / 2
def timeAccelUp(*args):
    global accel, accel_offset
    accel += 1
    accel_offset = accel / 2
def timeAccelDownFaster(*args):
    global accel, accel_offset
    accel -= 20
    accel_offset = accel / 2
def timeAccelUpFaster(*args):
    global accel, accel_offset
    accel += 20
    accel_offset = accel / 2
def timeAccelStop(*args):
    global accel, speed, accel_offset, accel_orig
    speed = 0
    accel = -accel_orig
    accel_offset = accel / 2
def timeAccelReset(*args): # set back to where the game started
    global accel, accel_orig, speed, accel_offset, currsdate, currstime, yyyyOrig, mmOrig, ddOrig, hhOrig, mnOrig
    fd = currsdate.split('-')
    ft = currstime.split(':')
    yyyyL = yyyyOrig - int(fd[0])
    mmL   = mmOrig   - int(fd[1])
    ddL   = ddOrig   - int(fd[2])
    hhL   = hhOrig   - int(ft[0])
    mnL   = mnOrig   - int(ft[1])
    speed = accel_orig
    accel = 0
    accel_offset = accel / 2
    set_offset( (60*60*hhL + 60*mnL + 28 * 24 * 60 * 60 * mmL + 24 * 60 * 60 * ddL + 365 * 24 * 60 * 60 * yyyyL) )

##############################################################################

def getSettingsValue(parser, section, option, value, boolean = False):
    if parser.has_option(str(section), str(option)):
        if boolean:
            return parser.getboolean(str(section), option)
        else:
            return parser.get(str(section), str(option))
    else:
        return str(value)

def appReadSettings():
    global settingsFilePath, documentSectionGeneral
    global gWrapDebugText, gAppsize, gHideBG, gAppShowOnStart, gEast, bTimeS, bTimeDone, gButtonH, gTracklightsRange, gTracklightsGLOW, gLoadSaveTracklightsRange, gLoadSaveTracklightsPerTrack
    global yyyyWanted, mmWanted, ddWanted, hhWanted, mnWanted, gButtonsAboveDateTime, gUpdateInterval, gExtraHeight, gUseSymbols, gUseScaling
    global lbChng, accel_orig, night__brightness_adjust, gMinimal
    global gAutoRain, gAutoWipers, gManuRain, gManuWipers
    global spinnerRainV, spinnerWipersV, spinnerSoundV
    global sol_config, raceinipath, racelogpath, weather_orig, gKeepPlanSettings, gResetWeatherOnPPFilter
    global ppFilterLast, ppFilterCurr, gMaximumValues, gShowSettings
    global bDynamicWeather, sol_weather_plan, plans_path, ps_path, sol_dummy
    global hotKeyToggleMain, hotKeyTogglePlan, hotKeyPlusBig, hotKeyMinusBig, hotKeyPlusSmall, hotKeyMinusSmall, hotKeyPlusTiny, hotKeyMinusTiny
    global gOffsetSmaller, gOffsetSmall, gOffsetBig, gEnableHotKeys, gModifier, gSeasonsActive
    global sPatchConfigParticles, sPatchConfigParticlesDef, gOrigSmokeAmount, gOrigSmokeThickness, sPatchTrackAdjustDef, sPatchTrackAdjust, sPatchWeatherFXiniDef, sPatchWeatherFXini
    global lastday, sCustomFontName, bUseCustomFont, bdoWIcheck, sol_controllercfg
    try:
        lastday = datetime.datetime(2019,1,1) # make sure year button gets day of week
        if not os.path.isfile(settingsFilePath):
            with codecs.open(settingsFilePath, 'w', 'utf_8', errors='ignore') as sf:
                sf.close
                ac.log("SolWeatherApp: Settings file created")
        settingsParser = configparser.ConfigParser()
        settingsParser.optionxform = str
        settingsParser.read(settingsFilePath)

        gOffsetBig = 60 * 60 * int(getSettingsValue(settingsParser, documentSectionGeneral, "TIME_OFFSET_BIG", "6", False))
        gOffsetSmall = 60 * int(getSettingsValue(settingsParser, documentSectionGeneral, "TIME_OFFSET_SMALL", "45", False))
        gOffsetSmaller = 60 * int(getSettingsValue(settingsParser, documentSectionGeneral, "TIME_OFFSET_TINY", "5", False))
        gWrapDebugText = getSettingsValue(settingsParser, documentSectionGeneral, "uiWrapDebugText", False, True)
        gHideBG = getSettingsValue(settingsParser, documentSectionGeneral, "uiHideBG", False, True)
        gAppShowOnStart = getSettingsValue(settingsParser, documentSectionGeneral, "uiShowOnStart", False, True)
        gAppsize = float(getSettingsValue(settingsParser, documentSectionGeneral, "uiSize", "1.0", False))
        gButtonH = int(getSettingsValue(settingsParser, documentSectionGeneral, "uiButtonH", "35", False))
        gButtonsAboveDateTime = getSettingsValue(settingsParser, documentSectionGeneral, "uiButtonsAboveDateTime", False, True)
        gUpdateInterval = float(getSettingsValue(settingsParser, documentSectionGeneral, "uiUpdateInterval", "0.25", False))
        gExtraHeight = int(float(getSettingsValue(settingsParser, documentSectionGeneral, "uiExtraHeight", "35", False)))

        bUseCustomFont = False
        sFont = getSettingsValue( settingsParser, documentSectionGeneral, "USEFONT", "default", False)
        if "Digital" in sFont:
            sCustomFontName="Digital-7"
            bUseCustomFont = True
        if "Consolas" in sFont:
            sCustomFontName="Consolas"
            bUseCustomFont = True
        if "Ticking" in sFont:
            sCustomFontName="Ticking Timebomb BB"
            bUseCustomFont = True
        if "Strait" in sFont:
            sCustomFontName="Strait"
            bUseCustomFont = True
        if "Signika" in sFont:
            sCustomFontName="Signika Negative"
            bUseCustomFont = True
        if "Overlock" in sFont:
            sCustomFontName="Overlock"
            bUseCustomFont = True

        gTracklightsRange = 100
        gTracklightsGLOW = 100
        gLoadSaveTracklightsRange = getSettingsValue(settingsParser, documentSectionGeneral, "uiLoadSaveTracklightsRange", False, True)
        gLoadSaveTracklightsPerTrack = getSettingsValue(settingsParser, documentSectionGeneral, "uiLoadSaveTracklightsPerTrack", False, True)

        bdoWIcheck = getSettingsValue(settingsParser, documentSectionGeneral, "CHECKWEATHERIMPLEMENTATION", True, True)
        gUseSymbols = getSettingsValue(settingsParser, documentSectionGeneral, "uiUseSymbols", False, True)
        gUseScaling = getSettingsValue(settingsParser, documentSectionGeneral, "uiUseScaling", False, True)
        gAutoWipers = getSettingsValue(settingsParser, documentSectionGeneral, "uiAutoWipers", False, True)
        gManuWipers = getSettingsValue(settingsParser, documentSectionGeneral, "uiManuWipers", False, True)
        spinnerWipersV = float(getSettingsValue(settingsParser, documentSectionGeneral, 'uiWiperSpeed', '0', False) )
        gMinimal = getSettingsValue(settingsParser, documentSectionGeneral, "uiMinimal", False, True)
        gMaximumValues = int(getSettingsValue(settingsParser, documentSectionGeneral, "uiMaximumValues", "300", False))
        gEast = getSettingsValue(settingsParser, "BACK2FUT", "uiEast", False, True)
        if gEast==True:
            bTimeDone=False
        bTimeS = str(getSettingsValue(settingsParser, "BACK2FUT", "uiEastString", "delorean", False)).strip().lower().replace(' ','')

        yyyyWanted=int(getSettingsValue(settingsParser, "BACK2FUT", "uiYear", "1985", False))
        mmWanted=int(getSettingsValue(settingsParser, "BACK2FUT", "uiMonth", "10", False))
        ddWanted=int(getSettingsValue(settingsParser, "BACK2FUT", "uiDay", "26", False))
        hhWanted=int(getSettingsValue(settingsParser, "BACK2FUT", "uiHour", "1", False))
        mnWanted=int(getSettingsValue(settingsParser, "BACK2FUT", "uiMin", "5", False))
        gUseSymbols = getSettingsValue(settingsParser, documentSectionGeneral, "uiUseSymbols", gUseSymbols, True)
        gKeepPlanSettings = getSettingsValue(settingsParser, documentSectionGeneral, "uiKeepPlanSettings", gKeepPlanSettings, True)
        gResetWeatherOnPPFilter = getSettingsValue(settingsParser, documentSectionGeneral, "uiResetWeatherOnPPFilter", gResetWeatherOnPPFilter, True)
        gShowSettings = getSettingsValue(settingsParser, documentSectionGeneral, "uiShowMoreSettingsOnPlan", gShowSettings, True)
        ###############

        gEnableHotKeys   =     getSettingsValue(settingsParser, "SHORTCUTS", "EnableHotKeys"    , True, True)
        gModifier        = str(getSettingsValue(settingsParser, "SHORTCUTS", "uiModifier"       , gModifier       , False)).strip().upper()
        hotKeyToggleMain = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyToggleMain" , hotKeyToggleMain, False)).strip().upper()
        hotKeyTogglePlan = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyTogglePlan" , hotKeyTogglePlan, False)).strip().upper()
        hotKeyPlusBig    = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyPlusBig"    , hotKeyPlusBig   , False)).strip().upper()
        hotKeyMinusBig   = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyMinusBig"   , hotKeyMinusBig  , False)).strip().upper()
        hotKeyPlusSmall  = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyPlusSmall"  , hotKeyPlusSmall , False)).strip().upper()
        hotKeyMinusSmall = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyMinusSmall" , hotKeyMinusSmall, False)).strip().upper()
        hotKeyPlusTiny   = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyPlusTiny"   , hotKeyPlusTiny  , False)).strip().upper()
        hotKeyMinusTiny  = str(getSettingsValue(settingsParser, "SHORTCUTS", "hotKeyMinusTiny"  , hotKeyMinusTiny , False)).strip().upper()

        accel_orig           = int(float( appReadCFGValue(raceinipath, 'LIGHTING', 'TIME_MULT'               ,'1' )))
        night__brightness_adjust = float( appReadCFGValue(sol_config , ''        , 'night__brightness_adjust','0.5'))

        zw=""
        if bRainSettingsPresent:
            zw = str(appReadCFGValue(sol_config, '', 'weather__set_rain_automatically', 'true'))

        if sol_controllercfg!='' and os.path.isfile(sol_controllercfg):
            zw = str(appReadCFGValue(sol_controllercfg, '', 'weather__set_rain_automatically', 'true'))
        gAutoRain=False
        if zw=='true':
            gAutoRain = True

        zw=""
        if bRainSettingsPresent:
            zw = appReadCFGValue(sol_config, '', 'weather__set_rain_amount', '0.0')
        if sol_controllercfg!='' and os.path.isfile(sol_controllercfg):
            zw = appReadCFGValue(sol_controllercfg, '', 'weather__set_rain_amount', '0.0')
        if zw!="":
            spinnerRainV = int (float( zw ) * 100.0)
        else:
            spinnerRainV = 0
        gManuRain=False
        if spinnerRainV>=0 and gAutoRain==False:
            gManuRain = True

        bDynamicWeather = False
        stmp = appReadCFGValue(sol_weather_plan, '', 'use_dynamic_weather_plan','false')
        if stmp.lower()=='true':
            bDynamicWeather = True

        # read user value
        gOrigSmokeAmount=1.0
        itmp = int( float( appReadCFGValue(sPatchConfigParticles, 'SMOKE', 'HEATING_EXTRA', '-1') )*100 )
        if itmp!=-1000:
            gOrigSmokeAmount = itmp
        else:
            # read default value
            stmp = appReadCFGValue(sPatchConfigParticlesDef, 'SMOKE', 'HEATING_EXTRA', '1')
            if ';' in stmp:
                gOrigSmokeAmount = int( float( stmp.split(' ')[0]) * 100)
        if gOrigSmokeAmount>50000:
            gOrigSmokeAmount=50000

        # read user value
        gOrigSmokeThickness = 8000
        itmp = int( float( appReadCFGValue(sPatchConfigParticles, 'SMOKE_DEV', 'LIMIT', '-1') ) )
        if itmp!=-1:
            gOrigSmokeThickness = itmp
        else:
            # read default value
            stmp = appReadCFGValue(sPatchConfigParticlesDef, 'SMOKE_DEV', 'LIMIT', '8000')
            if ';' in stmp:
                gOrigSmokeThickness = int( float( stmp.split(' ')[0]))
        if gOrigSmokeThickness>50000:
            gOrigSmokeThickness=50000

        gSeasonsActive = False

        # read user var from documents\a... c...\cfg\extension\weather_fx.ini
        lSeasonsActiveWfx = appReadCFGValue(sPatchWeatherFXini, 'MISCELLANEOUS', 'USE_TRACK_SEASONAL_ADJUSTMENTS', '-1')
        if lSeasonsActiveWfx=='-1':
            # read default value from \a...c...\cfg\extension\weather_fx.ini
            lSeasonsActiveWfx = appReadCFGValue(sPatchWeatherFXiniDef, 'MISCELLANEOUS', 'USE_TRACK_SEASONAL_ADJUSTMENTS', '1')
            if len(lSeasonsActiveWfx)>1:
                lSeasonsActiveWfx = lSeasonsActiveWfx[0]

        # read user var from documents\a... c...\cfg\extension\track_adjustments.ini
        lgSeasonsActive = appReadCFGValue(sPatchTrackAdjust, 'SEASONS', 'ALLOW_ADJUSTMENTS', '-1')
        if lgSeasonsActive=='-1':
            # read default value from \a...c...\cfg\extension\track_adjustments.ini
            lgSeasonsActive = appReadCFGValue(sPatchTrackAdjustDef, 'SEASONS', 'ALLOW_ADJUSTMENTS', '1')
            if len(lgSeasonsActive)>1:
                lgSeasonsActive = lgSeasonsActive[0]
        if lSeasonsActiveWfx=='1' or lgSeasonsActive=='1':
            gSeasonsActive = True

        spinnerSoundV  = float(appReadCFGValue(sPatchTrackAdjustDef, 'AUDIO', 'VOLUME', '1.0') ) * 100.0
        spinnerSoundV  = float(appReadCFGValue(sPatchTrackAdjustDef, 'AUDIO', 'VOLUME', str(spinnerSoundV)) ) * 100.0

        ppFilterLast = ''
        ppFilterCurr = appGetOriginalFilterName()
        ppFilterLast = ppFilterCurr
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

def getLastWeatherFromLog():
    global weather_orig, onlinesession, raceinipath, last_weather
    try:
        weather_orig = ''
        onlinesession = False
        stmp = appReadCFGValue(raceinipath, 'REMOTE', 'ACTIVE','0')
        if stmp.strip()=='1':
            onlinesession = True
            filedata=''
            with codecs.open(racelogpath, 'r', 'utf_8', errors='ignore') as file:
                filedata = file.read()
            m = re.search(r"ACP_WEATHER_UPDATE:.*", filedata)
            # m = re.search(r"ACP_WEATHER_UPDATE:(?!.*ACP_WEATHER_UPDATE:)", filedata).group()
            if m:
                # from log: ACP_WEATHER_UPDATE: Ambient=25.000000 Road=31.000000 GRAPHICS=3_clear
                # ac.log("SolWeatherApp m1: " + str(m))
                # ac.console('SolWeatherApp m1: ' + str(m))
                v = str(m).split('GRAPHICS=')
                if len(v)>1:
                    weather_orig = v[1].strip()
            else:
                # from log: Loading custom weather: 3_clear
                m = re.search(r"Loading custom weather: .*", filedata).group()
                # ac.log("SolWeatherApp m2: " + str(m))
                # ac.console('SolWeatherApp m2: ' + str(m))
                if m:
                    v = str(m).split(':')
                    # ac.log("SolWeatherApp v: " + str(m))
                    # ac.console('SolWeatherApp v: ' + str(m))
                    if len(v)>1:
                        weather_orig = v[1].strip()
        else:
            weather_orig = appReadCFGValue(raceinipath, 'WEATHER', 'NAME','default')
        if '(' in weather_orig:
            weather_orig=str(weather_orig).split('(')[0]
        if last_weather=='':
            ac.log('... weather: ' + weather_orig + ' ...done')
        tmp=re.sub(r"sol_[0..9][0..9]_", "", weather_orig)  # delete comments
        # .replace(r"sol_[0..9][0..9]_", "")
        if weather_orig!='':
            if onlinesession:
                ac.setText(lbWeatherinfo, 'Online weather: "' + weather_orig + '"')
            else:
                ac.setText(lbWeatherinfo, 'CM weather: "' + weather_orig + '"')
        bBig=False
        last_weather=''
        for l in tmp:
            if not l.isnumeric() and l!='_' and l!=' ' and l!='-':
                if last_weather=='' or bBig==True:
                    last_weather=last_weather+str(l).upper()
                    bBig=False
                else:
                    last_weather=last_weather+l
            if l=='_':
                bBig=True
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console('SolWeatherApp error: ' + traceback.format_exc())

##############################################################################

def appGetOriginalFilterName():
    global app, INIvideo
    filter=''
    config = configparser.ConfigParser()
    config.optionxform = str
    config.read(INIvideo)
    for section in config:
        if section=="POST_PROCESS":
            for key, value in config.items(section):
                if key=="ENABLED" and value == "0":
                    filter = "DISABLED"
                if key=="FILTER":
                    filter = str(value)
    return filter

def createSpinner(appWindow, newSpinner, section, value, x, y, width, height, rangeMin, rangeMax, step, onValueChangeListener, visible = True):
    global spinnerNightBright, spinnerGlow
    newSpinner = ac.addSpinner(appWindow, "")
    ac.setPosition(newSpinner, x, y)
    ac.setSize(newSpinner, width, height)
    ac.setRange(newSpinner, rangeMin, rangeMax)
    ac.setStep(newSpinner, step)
    ac.setValue(newSpinner, value)
    ac.setVisible(newSpinner, visible)
    ac.addOnValueChangeListener(newSpinner, onValueChangeListener) # down below
    if section == "night__brightness_adjust":
        spinnerNightBright = newSpinner
    elif section == "tracklights_glow":
        spinnerGlow = newSpinner

def findInSubdirectory(filename, subdirectory=''):
    if subdirectory:
        path = subdirectory
    else:
        path = os.getcwd()
    for root, dirs, names in os.walk(path):
        if filename in names:
            return os.path.join(root, filename)
    return ''

def appGetTrackConfig():
    global sFileTrack, sFileTrack2, sTrack, sLayout
    sFileTrack = os.path.abspath('content/tracks/' + sTrack+'/extension/ext_config.ini')
    if not os.path.isfile(sFileTrack):
        sFileTrack = os.path.abspath('extension/config/tracks/loaded/' + sTrack+'.ini')
        if not os.path.isfile(sFileTrack):
            sFileTrack = os.path.abspath('extension/config/tracks/' + sTrack+'.ini')
            if not os.path.isfile(sFileTrack):
                sFileTrack = os.path.abspath('content/tracks/' + sTrack+'/'+sLayout+'/extension/ext_config.ini')
                if not os.path.isfile(sFileTrack):
                    sFileTrack = userDir + "/Assetto Corsa/cfg/extension/config/tracks/" +sTrack+'__'+sTrack+'.ini'
                    if not os.path.isfile(sFileTrack):
                        sFileTrack = userDir + "/Assetto Corsa/cfg/extension/config/tracks/" +sTrack+'__'+sLayout+'.ini'
                        if not os.path.isfile(sFileTrack):
                            sFileTrack = userDir + "/Assetto Corsa/cfg/extension/config/tracks/" + sTrack+'.ini'
                            if not os.path.isfile(sFileTrack):
                                sFileTrack = userDir + "/Assetto Corsa/cfg/extension/config/tracks/" + sTrack+'__'+sTrack+'.ini'
                                if not os.path.isfile(sFileTrack):
                                    sFileTrack = userDir + "/Assetto Corsa/cfg/extension/config/tracks/loaded/" +sTrack+'__'+sLayout+'.ini'
                                    if not os.path.isfile(sFileTrack):
                                        sFileTrack = userDir + "/Assetto Corsa/cfg/extension/config/tracks/loaded/" +sTrack+'.ini'
                                        if not os.path.isfile(sFileTrack):
                                            sFileTrack = os.path.abspath('extension/config/tracks/loaded/' + sTrack+'__'+sTrack+'.ini')
                                            if not os.path.isfile(sFileTrack):
                                                sFileTrack = os.path.abspath('extension/config/tracks/loaded/' + sTrack+'__'+sLayout+'.ini')
                                                if not os.path.isfile(sFileTrack):
                                                    sFileTrack = os.path.abspath('extension/config/tracks/loaded/' + sTrack+'__'+sTrack+'.ini')
                                                    if not os.path.isfile(sFileTrack):
                                                        sFileTrack = os.path.abspath('extension/config/tracks/' + sTrack+'__'+sLayout+'.ini')
                                                        if not os.path.isfile(sFileTrack):
                                                            sFileTrack = os.path.abspath('extension/config/tracks/' + sTrack+'__'+sTrack+'.ini')
                                                            if not os.path.isfile(sFileTrack):
                                                                sFileTrack = ''

def appGetTrackConfig2():
    global sFileTrack, sFileTrack2, sTrack, sLayout
    if os.path.isfile(sFileTrack): # now search for any secondary (maybe layout dependend) configs
        sFileTrack2 = os.path.abspath('content/tracks/' + sTrack+'/'+sLayout+'/extension/ext_config.ini')
        if sFileTrack!=sFileTrack2 and os.path.isfile(sFileTrack2):
            sFileTrack2=sFileTrack2 # not sure how to do it better
        else:
            sFileTrack2 = userDir + "/Assetto Corsa/cfg/extension/config/tracks/" +sTrack+'__'+sLayout+'.ini'
            if sFileTrack!=sFileTrack2 and os.path.isfile(sFileTrack2):
                sFileTrack2=sFileTrack2
            else:
                sFileTrack2 = userDir + "/Assetto Corsa/cfg/extension/config/tracks/loaded/" +sTrack+'__'+sLayout+'.ini'
                if sFileTrack!=sFileTrack2 and os.path.isfile(sFileTrack2):
                    sFileTrack2=sFileTrack2
                else:
                    sFileTrack2 = os.path.abspath('extension/config/tracks/loaded/' + sTrack+'__'+sLayout+'.ini')
                    if sFileTrack!=sFileTrack2 and os.path.isfile(sFileTrack2):
                        sFileTrack2=sFileTrack2
                    else:
                        sFileTrack2 = os.path.abspath('extension/config/tracks/' + sTrack+'__'+sLayout+'.ini')
                        if sFileTrack!=sFileTrack2 and os.path.isfile(sFileTrack2):
                            sFileTrack2=sFileTrack2
                        else:
                            sFileTrack2 = ''

def getGameInfo(all=False):
    global bPatchActive, bPause, settingsFilePath, sFileTrack, sFileCar, sFileTrack2, sFileCar2, sCar, sTrack, sLayout, app, lbChng, lbRang, userDir, carpath1, carpath2, lbCar
    global gOrigHeadLightRange, gOrigSmokeAmount, spinnerRangeHeadLights, spinnerSmokeAmount
    global spinner_LIT_MULT, spinner_CAR_LIGHTS_LIT_MULT, spinner_SPECULAR_MULT, spinnerRangeTrackLights, spinnerGlow
    global gLIT_MULT, gCAR_LIGHTS_LIT_MULT, gSPECULAR_MULT, errorMSG, gTracklightsRange, gTracklightsGLOW, gLoadSaveTracklightsRange, gLoadSaveTracklightsPerTrack
    global sPatchConfigLights, sPatchConfigParticles, sPatchConfigParticlesDef, sFileTrackShort, sFileCarShort
    global bTimeS, gEast
    # ac.setText(lbCar, str(sCar+".ini") + "\nheadlights range  m\n\nsmoke strength %\n\nsmoke thickness %")
    try:
        if not all:
            if bPatchActive:
                lightinfo = ''
                lightinfo =  str(ac.ext_getLightsVisible()) + '/' + str(ac.ext_getTrackLightsNum())
                if lbRang>0 and sFileTrack!='':  ### track config
                    if sLayout!='' and sTrack!=sLayout:
                        if gLoadSaveTracklightsRange:
                            if gLoadSaveTracklightsPerTrack:
                                ac.setText(lbRang, sFileTrackShort + ' - ' + lightinfo + ' vis.\n per track                           range %')
                            else:
                                ac.setText(lbRang, sFileTrackShort + ' - ' + lightinfo + ' vis.\n range %')
                        else:
                            ac.setText(    lbRang, sFileTrackShort + ' - ' + lightinfo + ' vis.')
                    else:
                        if gLoadSaveTracklightsRange:
                            if gLoadSaveTracklightsPerTrack:
                                ac.setText(lbRang, sFileTrackShort + ' - ' + lightinfo + ' vis.\n per track                           range %')
                            else:
                                ac.setText(lbRang, sFileTrackShort + ' - ' + lightinfo + ' vis.\n range %')
                        else:
                            ac.setText(    lbRang, sFileTrackShort + ' - ' + lightinfo + ' vis.')
                else: ### no track config
                    if gLoadSaveTracklightsRange:
                        if gLoadSaveTracklightsPerTrack:
                            ac.setText(lbRang, 'no track config, click = to create\n per track                           range %' )
                        else:
                            ac.setText(lbRang, 'no track config, click = to create\n range %' )
                    else:
                        ac.setText(lbRang, 'no track config, click = to create' )
        else:

            sTrack = ac.getTrackName(0)
            sLayout = ac.getTrackConfiguration(0)
            sCar = ac.getCarName(0)
            if (str(bTimeS).strip()=='') or not (bTimeS in sCar.replace(' ','')) or not (bTimeS in acinfo.static.carModel):
                gEast=False

            appGetTrackConfig()
            appGetTrackConfig2()

            sFileTrackShort = ""
            fshort = sFileTrack.replace('\\','/').split('/')
            fshort.reverse()
            for i in range(min(len(fshort),5)):
                sFileTrackShort = '\\' + str(fshort[i]) + sFileTrackShort

            sFileCar = os.path.abspath('content/cars/'+sCar+'/extension/ext_config.ini') # from car folder itself
            if not os.path.isfile(sFileCar):
                sFileCar = findInSubdirectory(sCar+'.ini', carpath1)    # from docs\ac\cfg\ext\cars
                if not os.path.isfile(sFileCar):
                    sFileCar = findInSubdirectory(sCar+'.ini', carpath2) # from ac\ext\cfg\cars
                    if not os.path.isfile(sFileCar):
                        sFileCar = carpath2 + '/mods/'+sCar+'.ini'

            if os.path.isfile(sFileCar):
                sFileCar2 = findInSubdirectory('ext_config.ini', carpath1)    # from docs\ac\cfg\ext\cars
                if sFileCar2!=sFileCar and sFileCar2!='':
                    sFileCar2=sFileCar2
                else:
                    sFileCar2 = findInSubdirectory(sCar+'.ini', carpath2) # from ac\ext\cfg\cars
                    if sFileCar2!=sFileCar and sFileCar2!='':
                        sFileCar2=sFileCar2
                    else:
                        sFileCar2=""

            #sFileCarShort = str(os.path.basename(sFileCar)).replace('.ini','')
            #sFileCarShort = sFileCarShort.replace('ext_config',sCar+'..\extcfg')
            sFileCarShort = ""
            fshort = sFileCar.replace('\\','/').split('/')
            fshort.reverse()
            for i in range(min(len(fshort),4)):
                sFileCarShort = '\\' + str(fshort[i]) + sFileCarShort

            # if os.path.isfile(sFileCar):
            if lbCar>0:
                # ac.setText(lbCar, str(sCar+".ini"))
                if not os.path.isfile( sFileCar ):
                    s='no config: ' + str(sCar) + ' - click = to create'
                    ac.setText(lbCar, s + '\nheadlights range %\n\nsmoke strength %\n\nsmoke thickness %')
                    #ac.setText(lbCar, str(sFileCar))
                else:
                    ac.setText(lbCar, sFileCarShort + '\nheadlights range %\n\nsmoke strength %\n\nsmoke thickness %')


            bPause=True # turn off reacting to slider-changes
            if gLoadSaveTracklightsRange==True:
                # read Range&Glow, default 1.0
                gTracklightsRange = int(float(appReadCFGValue(settingsFilePath, documentSectionGeneral, 'uiTracklightsRange', '1.0'))*100.0)
                gTracklightsGLOW = int(float(appReadCFGValue(settingsFilePath, documentSectionGeneral, 'uiTracklightsGLOW', '1.0'))*100.0)

                if gLoadSaveTracklightsPerTrack==True and os.path.isfile(sFileTrack):
                    # read from track config, default from above
                    # [SOL_WEATHERAPP]
                    # ; note: its temmporary, gets reset on patch reload
                    # TracklightsRange=1.0 ; Range of Tracklights, sets lights again at race start
                    # TracklightsGLOW=1.0 ; pre select Glow intensity Tracklights, sets it again at race start
                    gTracklightsRange = int(float( appReadCFGValue(sFileTrack, 'SOL_WEATHERAPP', 'TracklightsRange', str(gTracklightsRange/100) ))*100)
                    gTracklightsGLOW = int(float( appReadCFGValue(sFileTrack, 'SOL_WEATHERAPP', 'TracklightsGLOW', str(gTracklightsGLOW/100) ))*100)

                lastspinnerRangeTrackLights = gTracklightsRange
                lastspinnerGlow = gTracklightsGLOW
                if spinnerGlow>0:
                    ac.setValue(spinnerGlow, int(gTracklightsGLOW))
                    ac.setValue(spinnerRangeTrackLights, int(gTracklightsRange))
            if spinner_LIT_MULT>0:
                gLIT_MULT =  float( appReadCFGValue(sFileTrack, 'LIGHTING', 'LIT_MULT', '1.0') )
                ac.setValue(spinner_LIT_MULT, int(gLIT_MULT*100))
            if spinner_CAR_LIGHTS_LIT_MULT>0:
                gCAR_LIGHTS_LIT_MULT =  float( appReadCFGValue(sFileTrack, 'LIGHTING', 'CAR_LIGHTS_LIT_MULT', '1.0') )
                ac.setValue(spinner_CAR_LIGHTS_LIT_MULT, int(gCAR_LIGHTS_LIT_MULT*100))
            if spinner_SPECULAR_MULT>0:
                gSPECULAR_MULT =  float( appReadCFGValue(sFileTrack, 'LIGHTING', 'SPECULAR_MULT', '1.0') )
                ac.setValue(spinner_SPECULAR_MULT, int(gSPECULAR_MULT*100))
            if spinnerRangeHeadLights>0:
                tmp =  int( float( appReadCFGValue(sPatchConfigLights, 'BASIC', 'HEADLIGHTS_RANGE_MULT', '1.0') )*100.0 )
                if gOrigHeadLightRange==-1:
                    gOrigHeadLightRange = tmp
                    bPause=True # turn off reacting to slider-changes
                    ac.setValue(spinnerRangeHeadLights, int(tmp) ) # preset HeadLights from file
                    bPause=False # turn on reacting to slider-changes

            if spinnerSmokeAmount>0:
                #tmp = int( float( appReadCFGValue(sPatchConfigLights, 'BASIC', 'HEADLIGHTS_SATURATION_MULT', '0.9') )*100.0 )
                #tmp = int( float( appReadCFGValue(sPatchConfigLights, 'BASIC', 'LIGHT_FROM_BEHIND_MULT', '1.0') )*100.0 )
                # tmp = int( float( appReadCFGValue(sPatchConfigParticles, 'SMOKE_DEV', 'LIMIT', '8000') ) )
                bPause=True # turn off reacting to slider-changes
                ac.setValue(spinnerSmokeAmount, int(gOrigSmokeAmount)) # preset HeadLights from file
                bPause=False # turn on reacting to slider-changes

            if spinnerSmokeThickness>0:
                bPause=True # turn off reacting to slider-changes
                ac.setValue(spinnerSmokeThickness, int(gOrigSmokeThickness))
                bPause=False # turn on reacting to slider-changes

            bPause=False # turn on reacting to slider-changes
    except:
        ac.log('SolWeatherApp: '+ traceback.format_exc())

#######################

def appTrackLightsToggle(*args):
    global gLightsON, spinnerRangeTrackLights, spinnerGlow, lastspinnerRangeTrackLights, lastspinnerGlow
    gLightsON=False
    if str(args[1])=='1':
        gLightsON=True
        bPause=True
        ac.setValue(spinnerRangeTrackLights, lastspinnerRangeTrackLights)
        bPause=False
        ac.setValue(spinnerGlow, lastspinnerGlow) # triggers it to change all
    else:
        lastspinnerRangeTrackLights=ac.getValue(spinnerRangeTrackLights)
        lastspinnerGlow=ac.getValue(spinnerGlow)
        bPause=True
        ac.setValue(spinnerRangeTrackLights, 0)
        bPause=False
        ac.setValue(spinnerGlow, 0) # triggers it to change all

def appRangeHeadLights(value):
    global bPause, sFileCar, sCar, app, lbChng, gTracklightsRange, gTracklightsGLOW, gOrigHeadLightRange, gLightsON
    global spinnerRangeTrackLights, spinnerGlow, spinner_LIT_MULT, spinner_CAR_LIGHTS_LIT_MULT, spinner_SPECULAR_MULT
    global spinnerRangeHeadLights, sPatchConfigLights, gLoadSaveTracklightsRange, sPatchVersion
    if bPause==True:
        return
    try:
        bPause=True
        ac.ext_pauseFsWatching() #####
        appWriteCFGValue(sPatchConfigLights, 'BASIC', 'HEADLIGHTS_RANGE_MULT', str( float(value)/100.0 ), True)
        ac.ext_resumeFsWatching() #####
        # update lights, as they would be 100% again
        if gLoadSaveTracklightsRange==True:
            appApplyTrackConfig()
        bPause=False
    except:
        ac.log('SolWeatherApp: '+ traceback.format_exc())
        ac.console('SolWeatherApp: '+ traceback.format_exc())
        ac.setText(lbChng, '\n\n\n\n\n\nerror setting headlights\n'+ traceback.format_exc())

def appSmokeSettings(*args):
    global bPause, sFileCar, sCar, app, lbChng, gTracklightsRange, gTracklightsGLOW, gOrigHeadLightRange, gLightsON
    global spinnerSmokeAmount, spinnerSmokeThickness
    global sPatchConfigLights, gLoadSaveTracklightsRange, sPatchConfigParticles
    if bPause==True:
        return
    try:
        bPause=True
        ac.ext_pauseFsWatching() #####

        valueSA = ac.getValue(spinnerSmokeAmount)
        appWriteCFGValue( sPatchConfigParticles, 'SMOKE', 'HEATING_EXTRA', str( int(valueSA/100.0) ) , True)

        valueST = ac.getValue(spinnerSmokeThickness)
        appWriteCFGValue( sPatchConfigParticles, 'SMOKE_DEV', 'LIMIT', str( int(valueST) ) , True)
        if valueSA>0 and valueST>0:
            appWriteCFGValue( sPatchConfigParticles, 'SMOKE', 'ENABLED', '1', True)
        else:
            appWriteCFGValue( sPatchConfigParticles, 'SMOKE', 'ENABLED', '0', True)

        ac.ext_resumeFsWatching() #####
        appApplyTrackConfig()
        bPause=False
    except:
        ac.log('SolWeatherApp: '+ traceback.format_exc())
        ac.console('SolWeatherApp: '+ traceback.format_exc())
        ac.setText(lbChng, '\n\n\n\n\n\nerror setting headlights\n'+ traceback.format_exc())

def ps_smoke_settings(*args):
    global spinnerSmokeAmount, value, bPause
    if bPause: return
    # value=int(ac.getValue(spinnerSmokeAmount))
    # gOrigHeadLightRange = value
    appSmokeSettings()

def ps_range_headlights(*args):
    global spinnerRangeHeadLights, value, bPause
    if bPause: return
    value=int(ac.getValue(spinnerRangeHeadLights))
    # gOrigHeadLightRange = value
    appRangeHeadLights(value)

def ps_range_tracklights(*args):
    global spinnerRangeTrackLights, gTracklightsRange, bPause
    if bPause: return
    value=int(ac.getValue(spinnerRangeTrackLights))
    gTracklightsRange=value
    appApplyTrackConfig()

def onspinnerGlow(*args):
    global spinnerGlow, gTracklightsGLOW, bPause
    if bPause: return
    value=int(ac.getValue(spinnerGlow))
    gTracklightsGLOW = value
    appApplyTrackConfig()

def onspinnerNightBright(*args):
    global spinnerNightBright, sol_config
    val = ac.getValue(spinnerNightBright)
    appWriteCFGValue( sol_config, '', 'night__brightness_adjust', str(round(val/100, 2)))

##############################################################################

def ps_reset_weather(*args):
    global sol_dummy
    try:
        with codecs.open(sol_dummy, 'w', 'utf_8', errors='ignore') as f:# write
            f.write('\n')
    except:
        ac.log("SolWeatherApp error: " + sol_dummy + "\n" + traceback.format_exc())
        ac.console("SolWeatherApp error: " + sol_dummy + "\n" + traceback.format_exc())

def threadDoRunEditor(sf):
    os.startfile(os.path.normpath(sf))

def threadRunEditor(sf):
    global thread
    try:
        if not thread or not thread.is_alive():
            thread = threading.Thread(target=threadDoRunEditor, args=(sf,))
            thread.start()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console("SolWeatherApp error: " + traceback.format_exc())
def runEditor(sF):
    if os.path.isfile(sF):
        try:
            threadRunEditor('"' + os.path.abspath(sF) + '"')
        except:
            ac.log('SolWeatherApp - reading track config failed '+ traceback.format_exc())


def threadWaited2Secs():
    global lbChng, gLightsON
    time.sleep(1)
    getGameInfo(True)
    ac.setText(lbChng, '')
    # update lights, as they would be 100% again
    ac.ext_resumeFsWatching() #####
    appApplyTrackConfig()

def threadWait2Secs():
    global thread, lbChng
    try:
        if not thread or not thread.is_alive():
            ac.setText(lbChng, '---   waiting for lights to be set   ---')
            thread = threading.Thread(target=threadWaited2Secs, args=())
            thread.start()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console("SolWeatherApp error: " + traceback.format_exc())

def threadWaited5Secs():
    global lbChng, sol_weather_plan, lastfiledata, iTransitiontime
    time.sleep(iTransitiontime)
    ac.ext_pauseFsWatching()
    ac.setText(lbChng, '')
    with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as f:
        f.write(lastfiledata)
    ac.ext_resumeFsWatching()

def threadWait5Secs():
    global thread2, lbChng
    try:
        if not thread2 or not thread2.is_alive():
            ac.setText(lbChng, '---   waiting for weather to transition   ---')
            thread2 = threading.Thread(target=threadWaited5Secs, args=())
            thread2.start()
    except:
        ac.log("SolWeatherApp error: " + traceback.format_exc())
        ac.console("SolWeatherApp error: " + traceback.format_exc())

##############################################################################

def ps_force_weatherTIME():
    global sol_weather_plan, checkbox_ps_weather, plans, plans_path, spinner_ps_plan, bTimeDone
    global last_plan, last_weather, lbWeat
    try:
        if (bTimeDone):
            if last_weather!='':
                ps_force_weather(last_weather,1)
            elif last_plan!='':
                appSetPlan(last_plan)
                ac.setValue(spinner_ps_plan, plans.index(last_plan) )
        else:
            last_weather = ''
            last_plan = ''
            for k, v in checkbox_ps_weather.items(): # clear checkbox checked
                if ac.getValue(v):
                    last_weather = k.encode('ascii', 'ignore').decode('ascii').strip()
            if last_weather=='':
                last_plan = ac.getText(lb_spinner_ps_plan)

            if not 'TimeTravelPlan.lua' in plans and not os.path.isfile(plans_path+'TimeTravelPlan.lua'):
                # read conditions
                temperature_ambient = str(acinfo.physics.airTemp)
                temperature_road = str(acinfo.physics.roadTemp)
                wind_direction = str(acinfo.graphics.windDirection)
                wind_speed = str(random.randint(2, 15) if acinfo.graphics.windSpeed == 0 else round(acinfo.graphics.windSpeed,1))
                new_filedata = '-- created by sol_weather app ;)\n'
                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "Clear"'
                new_filedata += ', time_holding  = 0.05, time_changing = 0.05'
                new_filedata += ', temperature_ambient = 20, temperature_road = 20'
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = 0})'

                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "BrokenClouds"'
                new_filedata += ', time_holding  = 0.05, time_changing = 0.05'
                new_filedata += ', temperature_ambient = 20, temperature_road = 20'
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = 0})'

                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "HeavyThunderstorm"'
                new_filedata += ', time_holding  = 0.1, time_changing = 0.1'
                new_filedata += ', temperature_ambient = -20, temperature_road = -20'
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = 200})'

                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "BrokenClouds"'
                new_filedata += ', time_holding  = 0.05, time_changing = 0.05'
                new_filedata += ', temperature_ambient = 20, temperature_road = 20'
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = 0})'

                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "Hurricane"'
                new_filedata += ', time_holding  = 0.2, time_changing = 0.2'
                new_filedata += ', temperature_ambient = -20, temperature_road = -20'
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = 200})'

                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "HeavyThunderstorm"'
                new_filedata += ', time_holding  = 0.1, time_changing = 0.1'
                new_filedata += ', temperature_ambient = -20, temperature_road = -20'
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = 200})'

                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "BrokenClouds"'
                new_filedata += ', time_holding  = 0.05, time_changing = 0.05'
                new_filedata += ', temperature_ambient = 20, temperature_road = 20'
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = 0})'

                new_filedata += '__SOL_WEATHER_PLAN:add_weather_slot({weather = "' + "LightThunderstorm" + '"'
                new_filedata += ', time_holding  = 1000000, time_changing = -1'
                new_filedata += ', temperature_ambient = ' + str(temperature_ambient) + ', temperature_road = ' + str(temperature_road)
                new_filedata += ', wind_direction = ' + str(wind_direction) + ', wind_speed = ' + str(wind_speed) + '})'

                with codecs.open(plans_path+'TimeTravelPlan.lua', 'w', 'utf_8', errors='ignore') as file:
                    file.write(new_filedata)
                planSelectorAddPlans()

            i = plans.index('TimeTravelPlan.lua')
            if i>0:
                appSetPlan('TimeTravelPlan.lua')
                ac.setValue(spinner_ps_plan, i)
            else:
                ac.log('TimeTravelPlan.lua weather not found')
                ps_force_weather('Rain',1)
    except:
        ac.log('SolWeatherApp error: ' + traceback.format_exc())
        if lbWeat>0:
            ac.setText(lbWeat, traceback.format_exc())

def OriginalInit(sdate, stime):
    global yyyyOrig, mmOrig, ddOrig, hhOrig, mnOrig
    fd = sdate.split('-')
    ft = stime.split(':')
    yyyyOrig=int(fd[0])
    mmOrig  =int(fd[1])
    ddOrig  =int(fd[2])
    hhOrig  =int(ft[0])
    mnOrig  =int(ft[1])

def TimeInit(sdate, stime):
    global bTimeCount
    global yyyyL, mmL, ddL, hhL, mnL
    global yyyyWanted, mmWanted, ddWanted, hhWanted, mnWanted
    fd = sdate.split('-')
    ft = stime.split(':')
    if bTimeCount==0 and yyyyL==0 and mmL==0 and hhL==0 and mnL==0:
        yyyyL = yyyyWanted - int(fd[0])
        mmL   = mmWanted   - int(fd[1])
        ddL   = ddWanted   - int(fd[2])
        hhL   = hhWanted   - int(ft[0])
        mnL   = mnWanted   - int(ft[1])
    elif bTimeCount>=1:
        yyyyL = -yyyyL
        mmL = -mmL
        ddL = -ddL
        hhL = -hhL
        mnL = -mnL
        bTimeCount = 0

def TimeExec(delta_t, s):
    global bTimeDone, bTimeS, gEast, sCar, bTimeCount, gMinimal
    global yyyyL, mmL, ddL, hhL, mnL, stime
    global lbWeat, weathers
    if gEast:
        fSpeedCurr = float(acinfo.physics.speedKmh)*0.621371
        if bTimeDone==True and fSpeedCurr<0.1:
            bTimeDone=False
        if gMinimal==True:
            s = ''
        if bTimeDone==True:
            s = s + '\n\n☢ Flux∞Capacitor used !\n'
        else:
            s = s + '\n\n☢ Flux∞Capacitor ready, MPH : ' + str(round(fSpeedCurr,2)) +'\n'
        if gMinimal==True:
            ac.setText(lbTime, stime + '\n\n\n\n\n\n' + s
                +'↝ Destination in time:  '
                +str(hhWanted) +':'+ str(mnWanted) + '  ↭  '
                +str(yyyyWanted) +'-'+ str(mmWanted) +'-'+ str(ddWanted) +'  ↜\n↝  '
                +str(hhL) +' hrs '+ str(mnL) + ' mns  ↭  '
                +str(yyyyL) +' yrs '+ str(mmL) +' mts '+ str(ddL) +' dys  ↜' )
        else:
            ac.setText(lbWeat, s + "↝ Destination in time:  " +
                str(hhWanted) +':'+ str(mnWanted) + '  ↭  ' +
                str(yyyyWanted) +'-'+ str(mmWanted) +'-'+ str(ddWanted) +'  ↜\n↝  ' +
                str(hhL) +' hrs '+ str(mnL) + ' mns  ↭  ' +
                str(yyyyL) +' yrs '+ str(mmL) +' mts '+ str(ddL) +' dys  ↜'
                # +  '\n' + str(bTimeCount) + str(bTimeDone)
                )
        if (bTimeDone==False) and (fSpeedCurr>87.999):
            bTimeCount += 1
            bTimeDone = True
            set_offset(60*60*hhL + 60*mnL)
            ac.ext_weatherTimeOffset(      24 * 60 * 60 * ddL )
            ac.ext_weatherTimeOffset( 28 * 24 * 60 * 60 * mmL )
            ac.ext_weatherTimeOffset(365 * 24 * 60 * 60 * yyyyL)
            ps_force_weatherTIME()
            # ac.ext_takeAStepBack()
    else:
        ac.setText(lbWeat, s)

######################

def ps_sound(*args):
    global spinnerSound, spinnerSoundV, sPatchTrackAdjust
    spinnerSoundV = ac.getValue(spinnerSound)
    # ---no luck
    # appWriteCFGValue( sPatchTrackAdjust, 'AUDIO', 'VOLUME', str(round(spinnerSoundV/100, 2)))

def HandleRain(sWeatherName):
    try:
        global timer3, sRainAmounts
        global spinnerRain, spinnerWipers, spinnerSound
        global gAutoRain, gAutoWipers, gManuRain, gManuWipers, bPause, bBase
        TargetRain   = 0.0
        RainParams = ac.ext_rainParams() # get current rain/wipers state from patch

        # todo
        if timer2>1.0:
            bPause = True
            ac.setValue(spinnerRain, int(RainParams[0]*100.0))
            bPause = False

        # calc rain offset
        if gAutoRain:
            if sWeatherName!='' and sWeatherName in weathers:
                TargetRain   = sRainAmounts[ weathers.index(sWeatherName) ]
                # TargetRain   = RainParams[0]
                if TargetRain>RainParams[0]:
                    if TargetRain>0.1:
                        TargetRain = 0.05
                    else:
                        TargetRain = 0.1
                elif TargetRain<RainParams[0]:
                    TargetRain = -0.1
                else:
                    TargetRain = 0.0
            else:
                TargetRain   = 0.0
        elif gManuRain==True:
            TargetRain = float( ac.getValue(spinnerRain) ) / 100.0
            if TargetRain>RainParams[0]:
                TargetRain = 0.1
            elif TargetRain<RainParams[0]:
                TargetRain = -0.1
            else:
                TargetRain = 0
        # set rain it if we have to
        if (bBase or gManuRain) and TargetRain!=0:
            ac.ext_rainParamsAdjust(TargetRain, 0)
            #appWriteCFGValue(sol_config, '', 'weather__set_rain_automatically', '---'))
            #appWriteCFGValue(sol_config, '', 'weather__set_rain_amount', '0.0')

        TargetWipers = 0
        if gAutoWipers==True:   # calc wipers offset
            # TargetWipers = sWiperSpeeds[ weathers.index(sWeatherName) ]
            if RainParams[0]>=0.05 and RainParams[0]<0.25:
                TargetWipers = 1
            else:
                TargetWipers = int(round(RainParams[0]*4))
            bPause = True
            ac.setValue(spinnerWipers, int(RainParams[1]))
            bPause = False
            if TargetWipers>RainParams[1]:
                TargetWipers = 1
            elif TargetWipers<RainParams[1]:
                TargetWipers = -1
            else:
                TargetWipers = 0
        elif gManuWipers==True:
            TargetWipers = ac.getValue(spinnerWipers)
            if TargetWipers>RainParams[1]:
                TargetWipers = 1
            elif TargetWipers<RainParams[1]:
                TargetWipers = -1
            else:
                TargetWipers = 0

        # set it if we have to
        if TargetWipers!=0.0 and (gManuWipers or gAutoWipers):
            if timer3 > 1.5: # update slow for wiper steps
                timer3 = 0.0
                ac.ext_rainParamsAdjust(0, TargetWipers)
        s = sWeatherName
        if round(RainParams[0],3)!=0.0 or round(RainParams[1],3)!=0.0 or round(TargetRain,3)!=0.0 or round(TargetWipers,3)!=0.0:
            if sWeatherName!='':
                s = ' - ' + sWeatherName                                  + '\n > rain: '+str(round((RainParams[0]+TargetRain)*100, 3)) + '% -  wipers: ' + str(round((RainParams[1]+TargetWipers))) + '/4\n'
            else:
                s = '\n > rain: ' + str(round((RainParams[0]+TargetRain)*100, 3)) + '% -  wipers: ' + str(round((RainParams[1]+TargetWipers))) + '/4\n'
        return s
    except:
        ac.log('sol_weather app error: ' + sWeatherName + '\n' + traceback.format_exc()) # patch not active

######################

def appGetPatchState():
    global app, sAppError, bPatchActive, bwFXactive, sPatchVersion, sVer
    global lbWeat, sol_config, sVerCode
    sAppError=''
    bPatchActive = False
    bwFXactive = False
    try:
        sVerCode = ac.ext_patchVersionCode()
        sPatchVersion = 'SolWeatherApp v'+sVer+' - CSP v' + str(ac.ext_patchVersion()) + ' (' + str(sVerCode) + ')'
        #sPatchVersion = ' SolWeatherApp v'+sVer+' - patch v' + str(ac.ext_patchVersion()) + " - code " + str(sVerCode)
        bPatchActive = True
        if ac.ext_weatherFxActive()==True:
            bwFXactive = True
    except:
        sAppError = 'Shaders Patch not enabled or installed.'
        sPatchVersion = ' SolWeatherApp v' + sVer
        if app>0 and lbWeat>0:
            # ac.setText(lbWeat, sAppError) # only display error msg
            ac.setText(lbTime, sAppError) # only display error msg
    try:
        if bwFXactive:
            if not os.path.isfile(sol_config):
                sAppError=sAppError + '\n\n\n\nsol__config.lua not found, is Sol installed?'
        else:
            sAppError = sAppError + '\n\n\n\n\nWeatherFX disabled.'
            if lbWeat>0:
                ac.setText(lbTime, sAppError) # only display error msg
                # ac.setText(lbWeat, sAppError)
    except:
        return # only to prevent errors

def updatePath():
    global s_ImplementationDef, s_Implementation, s_ControllerDef, s_Controller, sol_config, sol_controllercfg
    global sol_dummy, ps_path, plans_path, sol_weather_plan, bRainSettingsPresent, bPause, bBase, bSol2
    s_ImplementationDef = appReadCFGValue(sPatchWeatherFXiniDef, 'BASIC', 'IMPLEMENTATION', 'sol')
    s_Implementation =    appReadCFGValue(sPatchWeatherFXini   , 'BASIC', 'IMPLEMENTATION', s_ImplementationDef)
    s_ControllerDef =     appReadCFGValue(sPatchWeatherFXiniDef, 'BASIC', 'CONTROLLER'    , 'sol')
    s_Controller =        appReadCFGValue(sPatchWeatherFXini   , 'BASIC', 'CONTROLLER'    , s_ControllerDef)

    sol_config              = 'extension/weather/'             + s_Implementation + '/sol__config.lua'
    sol_controllercfg       = 'extension/weather-controllers/' + s_Implementation + '/ctrl_config.lua'
    sol_dummy               = 'extension/weather/'             + s_Implementation + '/reset_dummy.lua'
    ps_path                 = 'extension/weather-controllers/' + s_Controller + '/'
    plans_path              = ps_path + 'weather_plans/'
    sol_weather_plan        = ps_path + 'SOL__WEATHER_PLAN.lua'

    bRainSettingsPresent = False
    if os.path.isfile(sol_config):
        s1=appReadCFGValue(sol_config, '', 'weather__set_rain_automatically', '---')
        s2=appReadCFGValue(sol_config, '', 'weather__set_rain_amount'       , '---')
        if s1!='---' and s2!='---':
            bRainSettingsPresent = True
    else:
        sol_config = ''

    if not os.path.isfile(sol_controllercfg):
        sol_controllercfg = ''

    bBase=False
    bSol2=False
    if 'base' in ps_path:
        bBase=True
    elif 'sol2' in s_Controller:
        bSol2=True
    elif 'sol' in s_Controller:
        if not bPause:
            planSelectorAddPlans()

def acUpdate(delta_t):
    global app, ps_app, lbWeat, timer, timer2, timer3, speed, day_offset, gWrapDebugText, gHideBG
    global accel_offset, accel, accel_orig, firstrun
    global lbTime, lbDate, lbMult, bwFXactive, dt_obj
    global onlinesession, lbWeatherinfo, weather_orig
    global gUpdateInterval, currsdate, currstime, gUseSymbols, gAutoRain, gPlanLoop
    global bPatchActive, sPatchVersion, sAppError, gMinimal, bDynamicWeather, bForcedWeather
    global ppFilterCurr, ppFilterLast, gResetWeatherOnPPFilter, racelogpath, errorMSG
    global lastwfxINIdate, sPatchWeatherFXiniDef, sPatchWeatherFXini
    global sol_config, sol_dummy, plans_path, ps_path, sol_weather_plan, bBase, sol_controllercfg
    global sdatetime, sdate , stime, smultacel, sWeatherName, sWeatherNameUpcoming, sRainWiper
    global timer4, lastday, timeOffsetMinusYearButton, bdoWIcheck

    sDebug=""
    sorig=""
    timer += delta_t
    timer2 += delta_t
    timer3 += delta_t
    timer4 += delta_t

    if timer2 > 1.3452346: # things to update slow
        timer2 = 0.0
        if gHideBG==True:
            ac.setBackgroundOpacity(app, 0)
            ac.setBackgroundOpacity(ps_app, 0)
        else:
            ac.setBackgroundOpacity(app, 0.5)
            ac.setBackgroundOpacity(ps_app, 0.5)
        appGetPatchState()

        if bPatchActive==True and bwFXactive==True:
            if ppFilterCurr!="DISABLED" and gResetWeatherOnPPFilter==True:
                try:
                    ppFilterCurr = ac.ext_currentPpFilter()
                    if ppFilterCurr != ppFilterLast:
                        if firstrun==False:
                            ps_reset_weather()
                            ac.console('SolWeatherFX: reset after ppFilter change to ' + str(ppFilterCurr))
                        ppFilterLast = ppFilterCurr
                except:
                    ac.setText(lbWeat, '\n\n\n\n\n\nError: \n\n' + traceback.format_exc()) # patch not active

            if bdoWIcheck==True:
                currentwfxINIdate = os.path.getmtime(sPatchWeatherFXini)
                if lastwfxINIdate != currentwfxINIdate:
                    lastwfxINIdate=currentwfxINIdate
                    updatePath()
                    appSetPositionsAndSizesPlan()
                    ac.log('SolWeatherApp: wfxINI changed - cfgs:\n' + sol_config +'\n'+  sol_dummy)
                    # ac.log('def Implementation: '+s_ImplementationDef + ' - now ' + s_Implementation)
                    # ac.log('def Controller    : '+s_ControllerDef + ' - now ' + s_Controller)
        return # skip the rest if we are here

    if bPatchActive==True and bwFXactive==True:
        day_offset += speed * delta_t
        if accel_offset!=0:
            ac.ext_weatherTimeOffset( accel_offset )
        if abs(day_offset) > 1:
            ac.ext_weatherTimeOffset( 24 * 60 * 60 * day_offset / abs(day_offset) )
            day_offset = 0

    if timer4 > gUpdateInterval: # things to update only every 250ms
        timer4 = 0.0
        if bPatchActive==True and bwFXactive==True:
            sDebug = ac.ext_weatherDebugText()

            if bVisiblePlan: # update lights info
                getGameInfo()

            vdatetime = re.findall(r"[\d]{1,4}-[\d]{1,2}-[\d]{2} [\d]{2}:[\d]{2}:[\d]{2}",sDebug)
            if len(vdatetime)>0:
                sorig=sDebug
                sdatetime=''
                sdate = ''
                stime = ''
                smultacel=''
                sWeatherName=''
                sWeatherNameUpcoming=''
                sRainWiper=''
                sdatetime = str(vdatetime).replace("['", "").replace("']", "")
                # current day: 2020-05-06 16:31:25
                f = sdatetime.split(' ')
                sdate = f[0]
                stime = f[1]

                if gEast==True:
                    getGameInfo() # do it
                    TimeInit(sdate, stime)
                    planON = ac.getValue(checkbox_ps_plan_enable)
                    if not planON and weather_orig!='':      # and not gKeepPlanSettings:
                        if onlinesession:
                            ac.setText(lbWeatherinfo, 'Online weather: "' + weather_orig + '"')
                        else:
                            ac.setText(lbWeatherinfo, 'CM weather: "' + weather_orig + '"')
                    else:
                        ac.setText(lbWeatherinfo, '')

                if currsdate=='':
                    getGameInfo(True)
                    OriginalInit(sdate, stime)

                currstime=stime
                currsdate=sdate
                dt_obj = datetime.datetime.strptime(sdate, '%Y-%m-%d')
                if lastday!=dt_obj:
                    lastday=dt_obj
                    if gUseSymbols==True:
                        ac.setText(    timeOffsetMinusYearButton, "y▾\n            "     + dt_obj.strftime('%A')) # -yr
                        if gButtonH>=25:
                            ac.setText(timeOffsetMinusYearButton, "▼\nyrs\n            " + dt_obj.strftime('%A')) # -yr
                    else:
                        ac.setText(    timeOffsetMinusYearButton, "-yr\n            "    + dt_obj.strftime('%A')) # -yr
                        if gButtonH>=25:
                            ac.setText(timeOffsetMinusYearButton, "-\nyrs\n            " + dt_obj.strftime('%A')) # -yr

                stime = '⌚' + stime
                if ac.ext_getAmbientMult()<0.95:
                    if gUseSymbols==True:
                        sdate = ' ✹ ' + sdate
                    else:
                        sdate = ' O ' + sdate
                else:
                    if gUseSymbols==True:
                        sdate = ' ☾ ' + sdate
                    else:
                        sdate = ' C ' + sdate
                if int(accel_orig + accel)==0 and int(speed)==0:
                    if gUseSymbols==True:
                        smultacel =     ' ⏳ time paused! use 1x'
                    else:
                        smultacel =     ' % time paused! use 1x'
                else:
                    if int(speed)==0:
                        if gUseSymbols==True:
                            smultacel = ' ⏳ mult: '  + str(round(accel_orig + accel,3)) + 'x'
                        else:
                            smultacel = ' % mult: '  + str(round(accel_orig + accel,3)) + 'x'
                    else:
                        if gUseSymbols==True:
                            smultacel = ' ⏳ '       + str(round(speed,3)) + ' days/s, mult: '  + str(round(accel_orig + accel,3)) + 'x'
                        else:
                            smultacel = ' % '        + str(round(speed,3)) + ' days/s, mult: '  + str(round(accel_orig + accel,3)) + 'x'

                vWeatherName = re.findall(r"Change\: W\. Current\: .*", sDebug)
                if len(vWeatherName)>0:
                    sWeatherName=vWeatherName[0].replace('Change: W. Current: ','').strip().replace(' ','')
                    if sWeatherName=='Overcast':
                        sWeatherName='OvercastClouds'
                    if gUseSymbols:
                        sWeatherName = weathers[ weathers.index(sWeatherName) ]

                    vWeatherNameUpcoming = re.findall(r"Change\: W\. Upcoming\: .*", sDebug)
                    if len(vWeatherNameUpcoming)>0:
                        sWeatherNameUpcoming = vWeatherNameUpcoming[0].replace('Change: W. Upcoming: ', '').strip().replace(' ','')
                        if sWeatherNameUpcoming=='Overcast':
                            sWeatherNameUpcoming='OvercastClouds'
                        if gUseSymbols:
                            sWeatherNameUpcoming = weathersNames[ weathers.index(sWeatherNameUpcoming) ]
                sRainWiper = HandleRain(sWeatherName)
        return # skip the rest if we are here

    if timer > gUpdateInterval and lbTime > 0: # things to update fast
        timer = 0.0
        if bPatchActive==True:
            try:
                # ac.setText(lbWeat, ac.ext_weatherDebugText() ) # original
                if firstrun==True:
                    # first when its default
                    firstrun=False
                    # getGameInfo(True)
                    getLastWeatherFromLog()

                if bwFXactive==True and bVisibleMain==True:
                    if timer4 != 0.0: # already fetched above?
                        sDebug = ac.ext_weatherDebugText()
                    s = sDebug
                    if gWrapDebugText == True:
                        s = '\n'.join(s.strip() for s in re.findall(r'.{1,80}(?:\s+|$)', s))
                        # s = s.replace('>>> Sol weather: v', '\n>>> Sol weather: v')
                    s = s.replace(' \n \n', '').strip()
                    s = s.replace('\n\n', '').strip()
                    s = s + '\n' + sPatchVersion
                    if gMinimal==True:
                        # ac.log(stime + ' - ' + sdate)
                        if '> rain: 0.0% -  wipers: 0/4' in sRainWiper:
                            stime = stime + sdate + smultacel + ' - ' + sWeatherName
                        else:
                            stime = stime + sdate + smultacel + sRainWiper
                        # if errorMSG!='':  # deactivated!
                        #     stime = stime + '\nDOUBLE config ! :\n' + errorMSG + ' -vs-\n' + sFileTrack
                        if gEast==True:
                            TimeExec(delta_t, stime)  # finally show something
                        else:
                            ac.setText(lbTime, stime)  # finally show something
                        ac.setText(lbWeat, '') # gMinimal -> hidden, no need to set
                    else:
                        if '> rain: 0.0% -  wipers: 0/4' in sRainWiper:
                            s = s.replace('Weather slot:', sWeatherName + ' - Weather slot:')
                        else:
                            s = s.replace('Weather slot:', sRainWiper + ' - Weather slot:')
                        # s = s + sRainWiper
                        ac.setText(lbTime, stime)
                        ac.setText(lbDate, sdate)
                        ac.setText(lbMult, smultacel)

                        # if errorMSG!='':  # deactivated!
                        #     s = s + '\nDOUBLE config renamed! :\n' + errorMSG + ' -vs-\n' + sFileTrack
                        if gEast==True:
                            TimeExec(delta_t, s) # finally show something
                        else:
                            ac.setText(lbWeat, s)  # finally show something
            except:
                ac.log('sol_weather app error: ' + traceback.format_exc()) # patch not active
                if lbWeat>0:
                    ac.setText(lbWeat, '\n\n\n\n\n\nError: \n\n' + traceback.format_exc()) # patch not active

def acMain(ac_version):
    global app, ps_app, gAppShowOnStart, gHideBG
    global bPatchActive, sAppError, lbWeat
    global gAutoRain, gAutoWipers, gManuRain, gManuWipers, weather_orig, onlinesession, lbWeatherinfo
    global checkbox_AutoRain, checkbox_AutoWipers, checkbox_ManualRain, checkbox_ManualWipers
    global bDynamicWeather, gKeepPlanSettings, bVisibleMain, gEnableHotKeys, bPause, bVisiblePlan
    global bUseCustomFont, sCustomFontName, sPatchVersion, checkbox_ps_plan_enable
    try:
        appGetPatchState()
        updatePath()
        appReadSettings()
        if bUseCustomFont:
            ac.initFont(0, sCustomFontName, 0, 0)

        appCreateMyself()
        appCreatePlanSelector()
        appSetPositionsAndSizes()
        appSetPositionsAndSizesPlan()

        bPause=True # turn off reacting to changes by myself
        if gAppShowOnStart==True:
            ac.setVisible(app,1)
            bVisibleMain=True
        if gAutoRain==True:
            ac.setValue(checkbox_AutoRain,1)
            ps_AutoRain(checkbox_AutoRain,1)
            ac.setText(checkbox_AutoRain, 'auto rain')
        elif gManuRain==True:
            ac.setValue(checkbox_ManualRain,1)
            ps_ManualRain(checkbox_ManualRain,1)
            ac.setText(checkbox_AutoRain, 'manual rain')
        if gAutoWipers==True:
            ac.setValue(checkbox_AutoWipers,1)
            ps_AutoWipers(checkbox_AutoWipers,1)
            ac.setText(checkbox_AutoWipers, 'auto wipers')
        elif gManuWipers==True:
            ac.setValue(checkbox_ManualWipers,1)
            ps_ManualWipers(checkbox_ManualWipers,1)
            ac.setText(checkbox_AutoWipers, 'manual wipers')
        planSelectorAddPlans()
        if str(gEnableHotKeys)=='True':
            setHotKeys()
        try:
            bVisibleMain = ac.ext_isAppVisible("Sol Weather")
            bVisiblePlan = ac.ext_isAppVisible("Sol Plan Selector")
        except: # fails without patch, so just assume visible true at first
            bVisibleMain = True # do noting if patch with needed function is not availble
            bVisiblePlan = True # do noting if patch with needed function is not availble
        bPause=False # turn on reacting to changed by myself
    except:
        sAppError = sAppError + traceback.format_exc()
        ac.log("SolWeatherApp error: " + sAppError)
        ac.console("SolWeatherApp error: " + sAppError)
    return "sol_weather"

def acShutdown():
    global ps_app, gMinimal, gAppsize, bPause
    global settingsFilePath, documentSectionGeneral, gTracklightsRange, gTracklightsGLOW, gLoadSaveTracklightsRange, gLoadSaveTracklightsPerTrack
    global gAutoRain, gAutoWipers, gManuRain, gManuWipers, gKeepPlanSettings, gLightsON, gExtraHeight
    global spinnerRain, spinnerWipers, spinnerSound, sFileTrack, spinnerGlow, spinnerRangeTrackLights, lastspinnerRangeTrackLights, lastspinnerGlow
    global gLIT_MULT, gCAR_LIGHTS_LIT_MULT, gSPECULAR_MULT, userDir, sTrack, sLayout, gShowSettings
    # ps_set_plan()
    bPause = True
    if gLoadSaveTracklightsRange==True:
        if gLightsON:
            gTracklightsRange = ac.getValue(spinnerRangeTrackLights)
            gTracklightsGLOW = ac.getValue(spinnerGlow)
        else:
            gTracklightsRange = lastspinnerRangeTrackLights
            gTracklightsGLOW = lastspinnerGlow
        if gLoadSaveTracklightsPerTrack==True:
            # [SOL_WEATHERAPP]
            # TracklightsRange=1.0 ; Range of Tracklights, sets lights again at race start
            # TracklightsGLOW=1.0 ; pre select Glow intensity Tracklights, sets it again at race start
            if str(gTracklightsRange/100) != str(float( appReadCFGValue(sFileTrack, 'SOL_WEATHERAPP', 'TracklightsRange', '1.0') ) ) :
                appWriteCFGValue(sFileTrack, 'SOL_WEATHERAPP', 'TracklightsRange', str(float(gTracklightsRange)/100.0))
            if str(gTracklightsGLOW/100) != str(float( appReadCFGValue(sFileTrack, 'SOL_WEATHERAPP', 'TracklightsGLOW', '1.0') ) ) :
                appWriteCFGValue(sFileTrack, 'SOL_WEATHERAPP', 'TracklightsGLOW', str(float(gTracklightsGLOW)/100.0))
        else:
            appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiTracklightsRange', str(float(gTracklightsRange)/100.0), True)
            appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiTracklightsGLOW', str(float(gTracklightsGLOW)/100.0), True)

    if gKeepPlanSettings==False and not bBase and not bSol2:
        appWriteCFGValue(sol_weather_plan, '', 'use_dynamic_weather_plan', 'false')
        #with codecs.open(sol_weather_plan, 'w', 'utf_8', errors='ignore') as f: # write
        #    f.write('use_dynamic_weather_plan = false\n')

    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiSize', str(round(gAppsize,1)), True)
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiExtraHeight', str(int(gExtraHeight)))
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiMinimal', str(gMinimal))
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiAutoRain', str(gAutoRain))
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiAutoWipers', str(gAutoWipers))
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiManuRain', str(gManuRain))
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiManuWipers',str(gManuWipers))
    appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiShowMoreSettingsOnPlan',str(gShowSettings))

    if gManuRain==True:
        appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiRainAmount', str(ac.getValue(spinnerRain)/100.0))
    if gManuWipers==True:
        appWriteCFGValue(settingsFilePath, documentSectionGeneral, 'uiWiperSpeed', str(ac.getValue(spinnerWipers)))

    if not os.path.isfile(sFileTrack): # build cfg name if non existant
        sFileTrack = 'extension/config/tracks/' + sTrack+'.ini'
        if sLayout!='':
            sFileTrack = 'extension/config/tracks/' + sTrack+'_'+sLayout+'.ini'

    if str(gLIT_MULT) != str(float( appReadCFGValue(sFileTrack, 'LIGHTING', 'LIT_MULT', '1.0') ) ) :
        appWriteCFGValue(sFileTrack, 'LIGHTING', 'LIT_MULT'           , gLIT_MULT           , True)
    if str(gCAR_LIGHTS_LIT_MULT) != str(float( appReadCFGValue(sFileTrack, 'LIGHTING', 'CAR_LIGHTS_LIT_MULT', '1.0') ) ) :
        appWriteCFGValue(sFileTrack, 'LIGHTING', 'CAR_LIGHTS_LIT_MULT', gCAR_LIGHTS_LIT_MULT, True)
    if str(gSPECULAR_MULT) != str( float( appReadCFGValue(sFileTrack, 'LIGHTING', 'SPECULAR_MULT', '1.0') ) ) :
        appWriteCFGValue(sFileTrack, 'LIGHTING', 'SPECULAR_MULT'      , gSPECULAR_MULT      , True)

def isASCII(data):
    try:
        data.encode('ascii', 'ignore').decode('ascii').strip()
    except UnicodeDecodeError:
        return False
    return True

def find_str(s, char):
    index = 0
    if len(char)>0 and char in s:
        c = char[0]
        for ch in s:
            if ch == c:
                if s[index:index+len(char)] == char:
                    return index
            index += 1
    return -1

def appWriteCFGValue(path, section, valname, value, createNonExistant=False):
    try:
        if not os.path.isfile(path):
            if createNonExistant:
                with codecs.open(path, 'w', 'utf_8', errors='ignore') as f:
                    f.write('; created by sol_weather app\n['+section+']\n'+valname+'='+str(value)+'\n')
            return
        else:
            sEqual = ' = ' if '.lua' in str(path).lower() else '='
            idx = -1
            sectid = -1
            found = False
            currsection = ''
            t = ''
            s=''
            try:
                with codecs.open(path, 'r', 'utf_8', errors='ignore') as f:
                    s=f.read()
            except:
                with codecs.open(path, 'r', errors='ignore') as f:
                    s=f.read()
            if s!='':
                ss = str(s).split('\n')
                for sss in ss:
                    idx+=1
                    if find_str(sss, '--')!=0 and find_str(sss, ';')!=0 and find_str(sss, '#')!=0:
                        if find_str(sss, '[')==0 and (']' in sss):
                            currsection = sss.split('[')[1].split(']')[0]
                        if find_str(sss, '['+section+']')==0:
                            sectid = idx
                        if find_str(sss, valname)==0 and currsection==section:
                            # ss[idx] = valname + ' = ' + str(value)
                            found = True
                            if '--' in sss:
                                iFound = find_str(sss,'--')
                                if iFound<=len(sss):
                                    ss[idx] = valname + sEqual + str(value) + ' '+ sss[int(iFound):]   # slice sss
                            elif ';' in sss:
                                iFound = find_str(sss,';')
                                if iFound<=len(sss):
                                    ss[idx] = valname + sEqual + str(value) + ' '+ sss[int(iFound):]   # slice sss
                            elif '#' in sss:
                                iFound = find_str(sss,'#')
                                if iFound<=len(sss):
                                    ss[idx] = valname + sEqual + str(value) + ' '+ sss[int(iFound):]   # slice sss
                            else:
                                ss[idx] = valname + sEqual + str(value)
                    else:
                        ss[idx] = sss
                    t = t + ss[idx] + '\n'
            else:
                found = True
                t = '[' + section + ']\n' + valname + sEqual + str(value)+'\n'

            if found == False and sectid>=0: # insert into existing section
                found = True
                ss=t.split('\n')
                ss.insert(sectid+1, valname + sEqual + str(value))
                t= '\n'.join(s for s in ss)
            elif found == False:             # append new section
                found = True
                t = t[sectid+1:].strip() + '\n\n[' + section + ']\n' + valname + sEqual + str(value)+'\n'
            if found == True:                # now write
                with codecs.open(path, 'w', 'utf_8', errors='ignore') as f:
                    f.write(t.strip()+'\n')   # write
    except:
        ac.log("SolWeatherApp error: " + section + ' - ' + valname+'\n'+ str(path) + '\n' + traceback.format_exc())
        ac.console("SolWeatherApp error: " + section + ' - ' + valname+'\n'+str(path) + '\n' + traceback.format_exc())

def appReadCFGValue(path, section, valname, default):
    try:
        result=default
        sect = ''
        if os.path.isfile(path):
            goOn = False
            s = ''
            try:
                with codecs.open(path, 'r', 'utf_8', errors='ignore') as file:
                    s = file.read()
            except:
                with codecs.open(path, 'r', errors='ignore') as file:
                    s = file.read()
            ss = str(s).split('\n')
            for sss in ss: # for all lines
                sss=sss.strip()
                if find_str(sss, '--')!=0 and find_str(sss, ';')!=0 and find_str(sss, '#')!=0:
                    if ('[' in sss) and (']' in sss):
                        sect = sss
                    if (section=='' or sect=='['+section+']') and (valname in sss):
                        v = sss.replace(' = ', '=').split('=')
                        if len(v)>1:
                            if section=='' and '--' in v[1]:
                                w = v[1].split('--')
                                if len(w)>0:
                                    result = w[0].replace(' ', '')
                            elif ';' in v[1]:
                                w = v[1].split(';')
                                if len(w)>0:
                                    result = w[0].replace(' ', '')
                            else:
                                result = v[1]
    except:
        ac.log('SolWeatherApp: \n' + path + ' \n' + traceback.format_exc())
        ac.console('SolWeatherApp: \n' + path + ' \n' + traceback.format_exc())
    return result

##############################################################################

class ApplyTrackConfigFlags:
    Nothing = 0
    RestoreConditionsState = 1

def appApplyTrackConfig():
    global bPause, sFileTrack, app, lbChng, spinnerRangeTrackLights, spinnerGlow, checkbox_Tracklights
    global gLIT_MULT, gCAR_LIGHTS_LIT_MULT, gSPECULAR_MULT
    global gTracklightsRange, gTracklightsGLOW, gLightsON
    if bPause==True:
        return
    try:
        if gLoadSaveTracklightsRange==True:
            if not os.path.isfile(sFileTrack):
                getGameInfo(True)

            if os.path.isfile(sFileTrack):
                gTracklightsRange=int(ac.getValue(spinnerRangeTrackLights))
                gTracklightsGLOW=int(ac.getValue(spinnerGlow))

                ss = ['',]
                s = ''
                try:
                    with codecs.open(sFileTrack, 'r', errors='ignore') as file:
                        s = file.read()
                except:
                    try:
                        with codecs.open(sFileTrack, 'r', 'utf_8', errors='ignore') as file:
                            s = file.read()
                    except:
                        ac.log('reading track config failed:\n'+ sFileTrack + '\n' + traceback.format_exc())

                ss = str(s).split('\n')

                idx = 0
                ssL = len(ss)
                while idx<ssL:  # per section to active/deactivate
                    sss = ss[idx].strip().replace(' ', '')
                    if find_str(sss, '[')==0 and (']' in sss) and ( '[LIGHT_' in sss or '[LIGHT_SERIES_' in sss) and not 'LIGHT_POLLUTION' in sss:
                        sect = sss
                        bActiveLine=False
                        idx+=1
                        while idx<ssL and find_str(ss[idx], '[')==0 and not (']' in ss[idx]):
                            ss[idx]=ss[idx].strip().replace(' ','')
                            if find_str(ss[idx],'ACTIVE')>=0 and ('=' in ss[idx]):
                                bActiveLine=True
                                if gLightsON==True:
                                    ss[idx]='ACTIVE=1'
                                else:
                                    ss[idx]='ACTIVE=0'
                            idx+=1
                        if bActiveLine==False:  # maybe some dont have "ACTIVE=", then insert
                            if gLightsON==True:
                                ss.insert(idx, 'ACTIVE=1')
                            else:
                                ss.insert(idx, 'ACTIVE=0')
                            ssL=len(ss)
                            idx+=1
                    idx+=1
                # while end

                idx = 0
                ssL = len(ss)
                while idx<ssL:  # for all lines of track-config
                    sss = ss[idx].strip().replace(' ', '')
                    if find_str(sss, '--')==0 or find_str(sss, ';')==0 or find_str(sss, '#')==0:
                        del ss[idx]
                        ssL = len(ss)
                        idx-=1
                    else:
                        if find_str(sss,'RANGE=')>=0:  # set RANGE= line
                            v = sss.split('=')
                            if len(v)>1:
                                w = v[1].split(';')
                                if len(w)>1:
                                    v[1]=w[0].strip()
                                w = v[1].split('#')
                                if len(w)>1:
                                    v[1]=w[0].strip()
                                if v[1].isnumeric() and float(v[1])>5.0:
                                    ss[idx]='RANGE='+str( float(v[1]) * float(gTracklightsRange)/100.0 )

                        if find_str(sss,'=ksEmissive')>=0:  # set KEY_x=ksEmissive, or to be exact, VALUE_0=
                            v = sss.split('=')
                            if len(v)>1:
                                w = v[0].split('_')
                                if len(w)>0:
                                    sKey=w[1].strip()
                                    if sKey!='' and not 'ORIGINAL' in sKey:
                                        colorline=[]
                                        if 'VALUE_'+sKey in ss[idx+1]:
                                            colorline = ss[idx+1].replace(' ', '').split('=')
                                        elif 'VALUE_'+sKey in sLast:
                                            colorline = sLast.replace(' ', '').split('=')

                                        if len(colorline)>1:
                                            if ';' in colorline[1]:
                                                color=str(colorline[1]).split(';')[0].split(',')
                                            elif '#' in colorline[1]:
                                                color=str(colorline[1]).split('#')[0].split(',')
                                            else:
                                                color=str(colorline[1]).split(',')
                                            if len(color)>3:
                                                col=str(color[0])+','+str(color[1])+','+str(color[2])+',' + str( round(float(color[3]) * float(gTracklightsGLOW)/100.0,3) )
                                                ss[idx+1]='VALUE_'+sKey+'='+col
                                            elif len(color)>2:
                                                col=str( round( float(color[0]) * float(gTracklightsGLOW)/100.0,3)) + ','+ str( round(float(color[1]) * float(gTracklightsGLOW)/100.0, 3)) + ','+ str( round(float(color[2]) * float(gTracklightsGLOW)/100.0,3) )
                                                ss[idx+1]='VALUE_'+sKey+'='+ col
                                            elif len(color)>1:
                                                ss[idx+1]='VALUE_'+sKey+'='+ str( round(float(color[0]) * float(gTracklightsGLOW)/100.0,3)) + ','+ str( round(float(color[1]) * float(gTracklightsGLOW)/100.0,3) )
                                            elif len(color)>0:
                                                ss[idx+1]='VALUE_'+sKey+'='+ str( round(float(color[0]) * float(gTracklightsGLOW)/100.0,3) )
                    sLast=sss
                    idx+=1
                # end  while for all lines of track-config

                s = '\n'.join(sss for sss in ss)
                s=s.replace('ACTIVE=1\nACTIVE=1\n', 'ACTIVE=1\n')
                s=s.replace('ACTIVE=0\nACTIVE=0\n', 'ACTIVE=0\n')
                s = s.strip()+'\n'

                # s = str('[MATERIAL_ADJUSTMENT_0]\n'+
                #     'ACTIVE=1\n'+
                #     'MESHES=aaalightploes\n'+
                #     'KEY_0=ksEmissive\n'+
                #     'VALUE_0=150,255,255,3.5\n'+
                #     'VALUE_0_OFF=0, 0, 0\n'+
                #     'KEY_1=ksAlphaRef\n'+
                #     'VALUE_1=-193\n'+
                #     'VALUE_1_OFF=0, 0, 0\n'+
                #     'CONDITION=NIGHT_SMOOTH\n'+
                #     '\n'+
                #     '[INCLUDE]\n'+
                #     'INCLUDE=common/conditions.ini\n'+
                #     '\n'+
                #     '[CONDITION_0]\n'+
                #     'NAME=NIGHT_SMOOTH\n'+
                #     'INPUT = SUN\n'+
                #     'LUT = (|0=0|88=0|88=1|180=1)\n'+
                #     'LAG = 0.97\n')
                # s=s.replace('common/', 'extension/config/tracks/common/')
                # s=s.replace('common\\', 'extension/config/tracks/common/')

                ac.ext_applyTrackConfig(s, ApplyTrackConfigFlags.RestoreConditionsState)
                # debug
                # ac.ext_setClipboardData(s)

                #bPause=True
                #ac.ext_pauseFsWatching()
                #with codecs.open(sFileTrack+'_apply.ini', 'w', 'utf_8', errors='ignore') as f:
                #    f.write(s.strip()+'\n')   # write
                #ac.ext_resumeFsWatching()
                #ac.setValue(checkbox_Tracklights,1)
                #bPause=False

        else:
            if str(gLIT_MULT) != str(float( appReadCFGValue(sFileTrack, 'LIGHTING', 'LIT_MULT', '1.0') ) ) :
                appWriteCFGValue(sFileTrack, 'LIGHTING', 'LIT_MULT'           , gLIT_MULT           , True)
            if str(gCAR_LIGHTS_LIT_MULT) != str(float( appReadCFGValue(sFileTrack, 'LIGHTING', 'CAR_LIGHTS_LIT_MULT', '1.0') ) ) :
                appWriteCFGValue(sFileTrack, 'LIGHTING', 'CAR_LIGHTS_LIT_MULT', gCAR_LIGHTS_LIT_MULT, True)
            if str(gSPECULAR_MULT) != str( float( appReadCFGValue(sFileTrack, 'LIGHTING', 'SPECULAR_MULT', '1.0') ) ) :
                appWriteCFGValue(sFileTrack, 'LIGHTING', 'SPECULAR_MULT'      , gSPECULAR_MULT      , True)
    except:
        ac.log('reading track config failed '+ traceback.format_exc())
        ac.setText(lbChng, '\n\n\n\n\n\n\n\n\n\n\n\n\n\nreading config failed\n'+ traceback.format_exc())


import ac, math






##############################################################################
# SOURCE:
#
# http://pysolar.org/
# 
# converted by Peter Boese
##############################################################################

earth_radius = 6378140.0  # meters
earth_axis_inclination = 23.45  # degrees
seconds_per_day = 86400


def get_declination(day):
    global earth_axis_inclination

    '''The declination of the sun is the angle between
    Earth's equatorial plane and a line between the Earth and the sun.
    The declination of the sun varies between 23.45 degrees and -23.45 degrees,
    hitting zero on the equinoxes and peaking on the solstices.
    '''

    return earth_axis_inclination * math.sin((2 * math.pi / 365.0) * (day - 81))

def equation_of_time(day):
    # "returns the number of minutes to add to mean solar time to get actual solar time."
    b = 2 * math.pi / 364.0 * (day - 81)
    return 9.87 * math.sin(2 * b) - 7.53 * math.cos(b) - 1.5 * math.sin(b)

def get_solar_time(longitude_deg, minutes, day):
    # "returns solar time in hours for the specified longitude and time," 
    # " accurate only to the nearest minute."
    return ( (minutes + 4 * longitude_deg + equation_of_time( day )) / 60 )

def get_hour_angle(longitude_deg, minutes, day):
    solar_time = get_solar_time(longitude_deg, minutes, day)
    return 15.0 * (solar_time - 12.0)



def get_altitude_azimuth(latitude_deg, longitude_deg, day, minutes):
# expect 19 degrees for solar.get_altitude(42.364908,-71.112828,datetime.datetime(2007, 2, 18, 20, 13, 1, 130320))

    declination_rad = math.radians(get_declination(day))
    latitude_rad = math.radians(latitude_deg)
    hour_angle = get_hour_angle(longitude_deg, minutes, day)

    first_term = math.cos(latitude_rad) * math.cos(declination_rad) * math.cos(math.radians(hour_angle))
    second_term = math.sin(latitude_rad) * math.sin(declination_rad)
    
    alt = math.degrees(math.asin(first_term + second_term))


    altitude_rad = math.radians(alt)
    hour_angle_rad = math.radians(hour_angle)

    azimuth_rad = math.asin(math.cos(declination_rad) * math.sin(hour_angle_rad) / math.cos(altitude_rad))
    azi = math.degrees(azimuth_rad)

    if math.cos(hour_angle_rad) < (math.tan(declination_rad) / math.tan(latitude_rad)):
        # abs(azimuth) bigger then 90°
        # convert result - is in the opposite quatrant 
        if azi > 0:
            azi = 180 - azi
        else:
            azi = -180 - azi    
       
    if latitude_deg < 0:
        if azi > 0:
            azi = 180 - azi
        else:
            azi = -180 - azi  
   
    return [alt, azi]


def get_altitude(latitude_deg, longitude_deg, day, minutes):

    declination_rad = math.radians(get_declination(day))
    latitude_rad = math.radians(latitude_deg)
    hour_angle = get_hour_angle(longitude_deg, minutes, day)

    first_term = math.cos(latitude_rad) * math.cos(declination_rad) * math.cos(math.radians(hour_angle))
    second_term = math.sin(latitude_rad) * math.sin(declination_rad)
    
    return math.degrees(math.asin(first_term + second_term))


def get_time_from_sunangle(longitude_deg, latitude_deg, day, sunangle, am_pm, timezone):
    
    act = -1
    last = -1

    if 'pm' in am_pm:
        for i in range(720, 1441):
            act = get_altitude(latitude_deg, longitude_deg, day, (i-timezone*60))
            if last > sunangle and act <= sunangle:
                return i
            last = act
    else:
        for i in range(0, 720):
            act = get_altitude(latitude_deg, longitude_deg, day, (i-timezone*60))
            if last < sunangle and act >= sunangle:
                return i
            last = act

    return -999 # sun is not reaching this angle for the given day



















class Stellar:

    def __init__(self, chgCallback):

        self.daySeconds = -1
        self.dayOfTheYear = -1

        self.timeZoneOffset = -999
        self.trackCoordinates = [-999, -999]

        self.changedCallback = chgCallback

    def setWorldPosition(self, long, lat):
        self.trackCoordinates = [long, lat]
        self.checkChanged()
        
    def setLongitude(self, long):
        self.trackCoordinates[0] = long
        self.checkChanged()

    def setLatitude(self, lat):
        self.trackCoordinates[1] = lat
        self.checkChanged()

    def setTime(self, sec):
        self.daySeconds = sec

    def setDayOfTheYear(self, day):
        self.dayOfTheYear = day
        #ac.log('day, %i'%day)
        self.checkChanged()

    def setTimeZoneOffset(self, offset):
        self.timeZoneOffset = offset/3600
        self.checkChanged()

    def setChangedCallback(self, func):
        self.changedCallback = func

    def checkChanged(self):

        if (self.dayOfTheYear>=0 and
        self.timeZoneOffset>=-12 and
        self.trackCoordinates[0]>=-180 and
        self.trackCoordinates[1]>=-90):

            if self.changedCallback:
                self.changedCallback(self)

    def getTimeFromSunangle(self, sunangle, am_pm):
        return get_time_from_sunangle(self.trackCoordinates[0], self.trackCoordinates[1], self.dayOfTheYear, sunangle, am_pm, self.timeZoneOffset)
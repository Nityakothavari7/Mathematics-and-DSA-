#Code to get coordinates
import sys
from astropy.coordinates import SkyCoord, AltAz, EarthLocation
from astropy.time import Time
import astropy.units as u

#  Sirius
object_name = sys.argv[1]

location = EarthLocation.of_site('greenwich')
time = Time.now()

coord = SkyCoord.from_name(object_name)
altaz = coord.transform_to(AltAz(obstime=time, location=location))
az = altaz.az.deg
el = altaz.alt.deg
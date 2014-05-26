# Eyes Xing eyesxing_msgs/OOI message
#
# Message delineates all Objects Of Interest in an image. 

Header    header      # standard header
string[]  id          # the object of interest string identifier
string[]  isa         # object is a type of thing string identifier
Rect[]    bbox        # bounding box circumscribing object, in pixel coordinates
float64[] confidence  # confidence of object identification 0.0 - 1.0

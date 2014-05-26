# Eyes Xing eyesxing_msgs/Classifier message
#
# Specifies object detection classifier.
#
string  name          # name tag associated with this classifier
string  filename      # classifier description and trained data file path name
uint8   setop         # set operator applied to set from upstream classifier(s)

# Defined set operators between upstrem detection set and this classifier's
# detection set.
#
# Given C = C_1 setop C_2 setop ... setop C_n_1 is the result of the upstream 
# classifiers, and C_n is this classifier, then the new set C' is constructed
# using one of the following set operators.
#
uint8 SETOP_UNION           = 0 # set of all objects in C or C_n
uint8 SETOP_INTERSECTION    = 1 # set of all objects in C and C_n
uint8 SETOP_REL_COMPLIMENT  = 2 # set of all objects in C but not in C_n

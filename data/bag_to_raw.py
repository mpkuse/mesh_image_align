import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2

bag = rosbag.Bag('banana.bag')
for topic, msg, t in bag.read_messages(topics=['/mynteye/right/image_raw']):
    # print msg
    stamp = str(msg.header.stamp)
    cv_image = CvBridge().imgmsg_to_cv2(msg )
    print cv_image.shape
    cv2.imshow( 'win', cv_image )
    cv2.waitKey(30)

    print stamp+'.jpg'
    cv2.imwrite( 'right/%s.jpg' %(stamp), cv_image )

bag.close()

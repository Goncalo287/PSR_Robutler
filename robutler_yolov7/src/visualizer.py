"""
This script draw rectangle and write label on an image.

Authors :
    Alexandre   FOUCHER         foucher@univ-ubs.fr         January 2023
"""

# ==============================================================================
#                                 I M P O R T S
# ==============================================================================

import numpy as np
import cv2 as cv
from typing import List, Union



# ==============================================================================
#                               C O N S T A N T E S
# ==============================================================================

BOX_COLOR = (0,0,255)
BOX_THICKNESS = 6
LABEL_MARGIN = 12

# ==============================================================================
#                                     C O D E
# ==============================================================================

def draw_detections(img: np.array, bboxes: List[List[int]], classes: List[int],
                    class_labels: Union[List[str], None]):
    
    label_list = []
    
    for bbox, cls in zip(bboxes, classes):
        x1, y1, x2, y2   = bbox
        height, width, _ = img.shape

        # Draw the projection on the camera output
        cv.rectangle(
            img,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            BOX_COLOR,
            BOX_THICKNESS
        )

        if class_labels:
            label = class_labels[int(cls)]

            # Get label text size
            text_size = cv.getTextSize(label,cv.FONT_HERSHEY_SIMPLEX,1.7,2)[0]

            # Write label on the camera output (the first putText is to make an outline effect)
            xpos = min(width-20-text_size[0], max(20, int(x1)))
            ypos = min(height-20-text_size[1], max(20, int(y2)))

            # Label background
            cv.rectangle(
                img,
                (xpos-LABEL_MARGIN, ypos),
                (xpos+text_size[0]+LABEL_MARGIN, ypos+text_size[1]+LABEL_MARGIN*2),
                BOX_COLOR,
                -1
            )
            # Print label
            cv.putText(
                img,
                label,
                (int(xpos+LABEL_MARGIN/2), int(ypos+text_size[1]+LABEL_MARGIN/2)),
                cv.FONT_HERSHEY_SIMPLEX,
                1.7,
                (255, 255, 255),
                2
            )
            
            label_list.append(label)
            
            

    return img, label_list 

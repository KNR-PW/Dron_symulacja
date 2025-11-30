import cv2
import sys

class OpenCVTrackerWrapper:
    def __init__(self, tracker_type='KCF'):
        self.tracker_type = tracker_type.upper()
        self.tracker = None
        self._create_tracker()

    def _create_tracker(self):
        """Factory to create OpenCV tracker based on type string."""
        # Mapping of tracker names to their creator functions
        # Note: Availability depends on OpenCV version and opencv-contrib-python
        
        # Try to access legacy trackers (OpenCV 4.5+)
        legacy = getattr(cv2, 'legacy', cv2)
        
        trackers = {
            'BOOSTING': getattr(legacy, 'TrackerBoosting_create', None),
            'MIL': getattr(cv2, 'TrackerMIL_create', getattr(legacy, 'TrackerMIL_create', None)),
            'KCF': getattr(cv2, 'TrackerKCF_create', getattr(legacy, 'TrackerKCF_create', None)),
            'TLD': getattr(legacy, 'TrackerTLD_create', None),
            'MEDIANFLOW': getattr(legacy, 'TrackerMedianFlow_create', None),
            'MOSSE': getattr(legacy, 'TrackerMOSSE_create', None),
            'CSRT': getattr(cv2, 'TrackerCSRT_create', getattr(legacy, 'TrackerCSRT_create', None)),
        }

        creator = trackers.get(self.tracker_type)

        if creator is None:
            print(f"Error: Tracker {self.tracker_type} not found or not available in this OpenCV version.")
            print(f"Available keys: {list(trackers.keys())}")
            # Fallback to KCF if possible, else raise error
            if trackers['KCF']:
                print("Falling back to KCF.")
                self.tracker = trackers['KCF']()
            else:
                raise RuntimeError(f"Could not create tracker {self.tracker_type}")
        else:
            self.tracker = creator()

    def init(self, frame, bbox):
        """
        Initialize the tracker with a frame and a bounding box.
        bbox format: (x, y, w, h)
        """
        # Re-create tracker instance to clear previous state
        self._create_tracker()
        return self.tracker.init(frame, bbox)

    def update(self, frame):
        """
        Update the tracker.
        Returns: (success, bbox)
        """
        if self.tracker is None:
            return False, None
        return self.tracker.update(frame)
# class Detection:
#     def __init__(self, bounding_box=(0, 0, 0, 0), color="", gps_pos=(0, 0)):
#         # Format x, y, w, h
#         self.bounding_box = bounding_box
#         self.color = color
#         self.gps_pos = gps_pos

#     def set_bounding_box(self, bb):
#         self.bounding_box = bb

#     def set_gps_pos(self, pos):
#         self.gps_pos = pos

#     def get_bounding_box(self):
#         return self.bounding_box

#     def get_gps_pos(self):
#         return self.gps_pos

#     def get_color(self):
#         return self.color

#     def add_image(self, image):
#         pass
class Detection:
    def __init__(self, bounding_box=(0, 0, 0, 0), color="", gps_pos=(0, 0), rel_gps=(0, 0)):
        # Format x, y, w, h
        self.bounding_box = bounding_box
        self.color = color
        self.gps_pos = gps_pos
        self.rel_gps = rel_gps

    def set_bounding_box(self, bb):
        self.bounding_box = bb

    def set_gps_pos(self, pos):
        self.gps_pos = pos

    def get_rel_gps(self):
        return self.rel_gps

    def get_gps_pos(self):
        return self.gps_pos

    def get_bounding_box(self):
        return self.bounding_box

    def get_color(self):
        return self.color

    def add_image(self, image):
        pass
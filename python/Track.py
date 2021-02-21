
class TrackCompute:

    def __init__(self, RefereeState):
        self.RefereeState = RefereeState

    def render(self):
        pass
    
    def compute(self):
        pass

    def update_car_position(self, x, y):
        pass


if __name__ == "__main__":
    from sampleTrack import sample
    print(sample)
    t = TrackCompute(sample)
    t.compute()
    t.render()
    t.update_car_pos(20, 20)
    t.render


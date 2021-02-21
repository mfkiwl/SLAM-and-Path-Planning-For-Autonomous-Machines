import matplotlib.pyplot as plt 
import json

def replaceAll(s, sub, rep):
    temp = str.maketrans(sub, rep)
    return s.translate(temp)

def sanetise_ref(s):
    s = s.replace("<RefereeState>", "").replace("<Point2D>", "")
    s = replaceAll(s, "'", '"')
    return s

def getColor(c):
    if c==0:
        return 'blue'
    elif c==1:
        return 'yellow'
    else:
        return 'red'

global ax1, ax2 
ax1 = plt.subplot(1,1,1)
# ax2 = plt.subplot(1,2,2)

plt.ion()
# plt.draw()
plt.show(block=False)

class TrackCompute:

    def __init__(self, RefereeState):
        self.rs = RefereeState
        self.car_pos = dict()
        # print(self.rs)

    def render(self):
        global ax1, ax2
        ax1.set_aspect('equal', adjustable='box')

        # plt.clf()
        for c in self.cones:
            ax1.plot(self.cones[c]["x"], self.cones[c]["y"], "o", color=getColor(c))
    
        self.car_pos.setdefault("x", False)
        self.car_pos.setdefault("y", False)
        if self.car_pos["x"] and self.car_pos["y"]:
            ax1.plot(self.car_pos["x"], self.car_pos["y"], "+")
        
        plt.draw()
        plt.pause(1)
        pass
    
    def compute(self):
        self.cones = dict()
        for cone in self.rs.cones: # ["cones"]:
            # cone['color'].setdefault(3)
            self.cones.setdefault(cone["color"], {"x": [], "y": []})
            self.cones[cone['color']]["x"].append(cone["x"])
            self.cones[cone['color']]["y"].append(cone["y"])
        pass

    def update_car_position(self, x, y):
        self.car_pos = {"x": x, "y": y}
        pass


if __name__ == "__main__":
    from sampleTrack import sample
    # print(sample)
    san = sanetise_ref(sample)
    parsed = json.loads(san)
    t = TrackCompute(parsed)
    t.compute()
    t.update_car_position(20, 20)
    t.render()


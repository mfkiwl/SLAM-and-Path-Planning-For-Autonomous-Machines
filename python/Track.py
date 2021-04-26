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
ax1 = plt.subplot(1,3,1)
ax2 = plt.subplot(1,3,2)
ax3 = plt.subplot(1,3,3)

plt.ion()
# plt.draw()
plt.show(block=False)

def distance(x1, y1, x2, y2):
    #print(x1, x2)
    return ( (x1-x2)**2 + (y1-y2)**2 )**0.5

class TrackCompute:

    def __init__(self, RefereeState, CarState):
        self.rs = RefereeState
        self.cs = CarState
        self.car_pos = dict()
        # print(self.rs)

    def render(self, CarState, imgL, imgR, imgD):
        self.cs = CarState
        global ax1, ax2, ax3
        ax3.imshow(imgL)
        ax1.set_aspect('equal', adjustable='box')
        # ax2.set_aspect('equal', adjustable='box')
        ax2.set_xlim([-10,10])
        ax2.set_ylim([-10,10])

        ax1.cla()
        # plt.clf()
        for c in self.cones:
            ax1.plot(self.cones[c]["x"], self.cones[c]["y"], "o", color=getColor(c))


        self.car_pos.setdefault("x", False)
        self.car_pos.setdefault("y", False)
        if self.car_pos["x"] and self.car_pos["y"]:
            ax1.plot(self.car_pos["x"], self.car_pos["y"], "o", color="r")
            ax2.cla()
            #ax2.imshow(self.cs)
            for c in self.cones:
                flt = {'x':[], 'y':[]}
                for i in range(len(self.cones[c]["x"])):
                    x = self.cones[c]["x"][i]
                    y = self.cones[c]["y"][i]
                    d = distance(x, y, self.car_pos['x'], self.car_pos['y'])
                    #print(d)
                    if d < 500:
                        flt['x'].append(x)
                        flt['y'].append(y)
                ax2.plot(flt["x"], flt["y"], "o", color=getColor(c))
            #ax2.title("CarState")
            #ax2.plot(0, 0, "o", color="r") 
            #ax2.plot(self.car_cones["x"], self.car_cones["y"], "o", color="b")
            #for c in self.cones:
            #    for i in range(len(self.cones[c]["x"])):
            #        near = {"x": [], "y": []}
            #        if distance(self.cones[c]["x"][i], self.cones[c]["y"][i], self.car_pos["x"], self.car_pos["y"] ) < 100:
            #            near["x"].append(self.cones[c]["x"][i])
            #            near["y"].append(self.cones[c]["y"][i])
            #        ax2.plot(near["x"], near["y"], "o", color=getColor(c))

        plt.draw()
        plt.pause(0.001)
        pass
    
    def compute(self):
        self.cones = dict()
        for cone in self.rs.cones: # ["cones"]:
            # cone['color'].setdefault(3)
            self.cones.setdefault(cone["color"], {"x": [], "y": []})
            self.cones[cone['color']]["x"].append(cone["x"])
            self.cones[cone['color']]["y"].append(cone["y"])
        pass

    def update_car_position(self, x, y, car_cones):
        self.car_pos = {"x": x, "y": y}
        self.car_cones = {"x": [], "y": []}
        for cone in car_cones:
            self.car_cones["y"].append(cone["x"])
            self.car_cones["x"].append(cone["y"])
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


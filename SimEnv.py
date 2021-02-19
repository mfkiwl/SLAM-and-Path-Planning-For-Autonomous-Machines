
class Space:
    def __init__(self, space):
        self.space = space
        pass

    def sample(self):
        # TODO Return random sample
        return self.space[0]

class Env:
    def __init__(self,):
        self.action_space
        pass

    def reset(self):
        # TODO Return state
        return self.state

    def step(self, action):
        # TODO Check if action is in action_space
        # TODO return new_state, reward, done, info

class VideoProcessingLayer:
    def __init__(self):
        self.enabled = True
        pass

    def is_enabled(self):
        return self.enabled

    def enable(self, state):
        self.state_changed(state)
        self.enabled = state

    def state_changed(self, new_state):
        pass
    
    def setup(self, ctx):
        pass
    
    def process(self, ctx):
        pass
    
    def release(self, ctx):
        pass

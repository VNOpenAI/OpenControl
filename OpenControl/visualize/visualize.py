import numpy as np
import os
from torch.utils.tensorboard import SummaryWriter

class Logger(object):
    """Real-time visualize simulation results, using Tensorboard
    
        Attributes:
            writer (class): the same as SummaryWriter_
            
        .. _SummaryWriter: https://pytorch.org/docs/stable/tensorboard.html
    """
    def __init__(self, log_dir='results', filename_suffix=''):
        """The same as SummaryWriter_ 

        Args:
            log_dir (str, optional): the direction to save the log file. Defaults to 'results' folder.
            filename_suffix (str, optional): suffix added to the name of log file. Defaults to ''.
        """
        if not os.path.isdir(log_dir):
            os.mkdir(log_dir)
        self.writer = SummaryWriter(log_dir=log_dir, filename_suffix=filename_suffix)
        
    def log(self, section, signals, step):
        """Log the signals with the name of section in the step time

        Args:
            section (str): name of signals
            signals (array): signals
            step (int): only `int` type is acceptable. Please convert `float` timestep to `int`
        """
        signals = np.array(signals).flatten()
        sig_dict = {}
        for i in range(len(signals)):
            sig_dict['_'+str(i)] = signals[i]
        self.writer.add_scalars(main_tag=section, tag_scalar_dict=sig_dict, global_step=step)
        
    # def log_series(self, section, series, steps):
    #     # series.shape = [n_series, n_signal]
    #     series = np.array(series)
    #     sample_time = (steps[-1] - steps[0])/(len(steps)-1)
    #     steps = (steps/sample_time).astype(int)
    #     for i in range(len(series)):
    #         self.log(section, series[i], steps[i])
            
    def end_log(self):
        """Call this function to end logging
        """
        self.writer.close()
        
if __name__ == '__main__':
    logX = Logger(log_dir='results', filename_suffix='step1')
    for i in range(100):
        logX.log('euler', [i*np.cos(i), i*np.sin(i)], i)
    logX.end_log()
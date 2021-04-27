import train_model
import argparse

parser = argparse.ArgumentParser(description='Training a DQN agent to play CarRacing.')
parser.add_argument('-m', '--model', help='Specify the last trained model path if you want to continue training after it.')
parser.add_argument('-s', '--start', type=int, help='The starting episode, default to 1.')
parser.add_argument('-e', '--end', type=int, help='The ending episode, default to 1000.')
parser.add_argument('-p', '--epsilon', type=float, default=1.0, help='The starting epsilon of the agent, default to 1.0.')
parser.add_argument('-exe', '--executable', type=str, default="../../Apps/fsds-v2.0.0-linux/FSDS.sh", help='Path to FSDS.sh')
parser.add_argument('-r', '--resume', default=False, action='store_true', help='Will resume training from latest save')
args = parser.parse_args()

train_model.train(args)
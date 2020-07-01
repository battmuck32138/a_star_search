
from game import GameStateData
from game import Game
from game import Directions
from game import Actions
from util import nearestPoint
from util import manhattanDistance
import searchAgents
import util, layout
import sys, types, time, random, os

lo = layout.getLayout("tinyCorners")
cp = searchAgents.CornersProblem()



from captureAgents import CaptureAgent
import distanceCalculator
import random
import time
import util
import sys
from game import Directions
import game
from util import nearestPoint

from dangerMap import DangerMap

class AttackSafeAgent(CaptureAgent):
  """
  An attack class that takes into account the danger map to always survive when against only one defender
  """

  distances = [10,10,10,10]
  index = 0

  def __init__(self, index, timeForComputing=.1):
    # Agent index for querying state
    self.index = index
    AttackSafeAgent.index = index
    # Whether or not you're on the red team
    self.red = None
    # Agent objects controlling you and your teammates
    self.agentsOnTeam = None
    # Maze distance calculator
    self.distancer = None
    # A history of observations
    self.observationHistory = []
    # Time to spend each turn on computing maze distances
    self.timeForComputing = timeForComputing
    # Access to the graphics
    self.display = None

  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.dangerMap = DangerMap(
        gameState.data.layout.walls, self.getMazeDistance)
    self.opponentsIndexes = self.getOpponents(gameState)
    print(self.dangerMap.getDangerMap())

  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    actions = gameState.getLegalActions(self.index)
    actions.remove('Stop')

    # You can profile your evaluation time by uncommenting these lines
    # start = time.time()
    values = [self.evaluate(gameState, a) for a in actions]
    # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]

    foodLeft = len(self.getFood(gameState).asList())

    if foodLeft <= 2:
      bestDist = 9999
      for action in actions:
        successor = self.getSuccessor(gameState, action)
        pos2 = successor.getAgentPosition(self.index)
        dist = self.getMazeDistance(self.start, pos2)
        if dist < bestDist:
          bestAction = action
          bestDist = dist
      return bestAction

    return random.choice(bestActions)

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def evaluate(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    AttackSafeAgent.distances = gameState.getAgentDistances()
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)
    state_reward = features * weights
    numCarrying = gameState.getAgentState(self.index).numCarrying
    if self.red:
      riskOfCarrying = 0.1 * numCarrying**2 * (gameState.getAgentPosition(self.index)[0] - 14)**2
    else:
      riskOfCarrying = 0.1 * numCarrying**2 * (17 - gameState.getAgentPosition(self.index)[0])**2
    return state_reward - riskOfCarrying

  def getFeatures(self, gameState, action):
    
    
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    myPos = successor.getAgentState(self.index).getPosition()
    foodList = self.getFood(successor).asList()
    features['successorScore'] = -len(foodList)  # self.getScore(successor)

    # Compute distance to the nearest food

    if len(foodList) > 0:  # This should always be True,  but better safe than sorry
      minDistance = min([self.getMazeDistance(myPos, food)
                         for food in foodList])
      features['distanceToFood'] = minDistance
    
    # Compute danger at agent position

    if (myPos[0] <= 15 and gameState.isOnRedTeam(self.index)) or (myPos[0] >= 16 and not gameState.isOnRedTeam(self.index)):
        features['danger'] = 0
    else:
        features['danger'] = self.dangerMap.getDanger(myPos)

    # Compute a metric representing how close is the closest ennemy

    distances = gameState.getAgentDistances()
    min_dist = 100
    scared = False
    for ennemy_index in self.opponentsIndexes:
        ennemy_pos = gameState.getAgentPosition(ennemy_index)
        if ennemy_pos != None:
            dist = self.getMazeDistance(ennemy_pos,myPos)
        else:
            dist = distances[ennemy_index]
        if dist < min_dist:
            min_dist = dist
        if gameState.getAgentState(ennemy_index).scaredTimer > 0:
          scared = True
        
    if min_dist > 5 or myPos[0] <= 15 or scared:
        features['ennemyProximity'] = 0
    elif min_dist == 5:
        features['ennemyProximity'] = 10
    elif min_dist == 4:
        features['ennemyProximity'] = 30
    elif min_dist == 3:
        features['ennemyProximity'] = 50
    elif min_dist == 2:
        features['ennemyProximity'] = 100
    elif min_dist == 1:
        features['ennemyProximity'] = 1000
    elif min_dist == 0:
        features['ennemyProximity'] = 1000000

    return features

  def getWeights(self, gameState, action):
    return {'successorScore': 10, 'distanceToFood': -1, 'danger': -3, 'ennemyProximity': -0.5}

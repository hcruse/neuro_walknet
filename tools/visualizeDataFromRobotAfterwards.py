from controller.DataPlayer.PEPVisualizationPlayer import PEPVisualizationPlayer
pepVisualization = PEPVisualizationPlayer( "/Users/mschilling/Desktop/PostureData/Hector/reaCog_walking_01.out" )

# Plot the Data
from controller.DataPlayer.PositionVisualizationPlayer import PositionVisualizationPlayer
posVisualization = PositionVisualizationPlayer( "/Users/mschilling/Desktop/PostureData/Hector/reaCog_walking_01.out" )

# Plot the Data
from controller.DataPlayer.CoGVisualizationPlayer import CoGVisualizationPlayer
cogVisualization = CoGVisualizationPlayer( "/Users/mschilling/Desktop/PostureData/Hector/reaCog_walking_01.out" )

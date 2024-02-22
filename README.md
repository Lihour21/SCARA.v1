# SCARA.v1
GUI:
- PanelRobot.fig. MatLab GUI to controling a 5R symetrical parallel mechanism. There is a bug wich prevents to open the GUI directly. It is necessary to execute from the command window: >>PanelRobot
- PanelRobot.m. MatLab script corresponding to PanelRobot.fig.
Paths for inverse kinematics:
- Flamenco.txt (Flamingo)
- Gallo.txt (Rooster)
- mano.t/xt (Hand)
- MatLabLogo.txt (MatLab's logo)
- Pinguino.txt (Penguin)
- poligono.txt (Poligon)

Table for forward kinematics (drive/r's angles)
- angulos.txt

Functions to process images and paths (.png, .mat, .txt)
- crearDibujo.m. Main function
- circles.m. plots circles of radius r at points x and y. Chad Greene, March 2014. University of Texas Institute for Geophysics.
- fiveRmic.m. Maximal Inscribed Circle within the workspace of a 5R symetrical parallel mechanism.

5R Parallel Robot GUI by:

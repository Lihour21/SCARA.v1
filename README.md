# SCARA.v1
GUI:
- ScaraRobot.fig. MatLab GUI to controling a 5-Bar symetrical parallel mechanism. There is a bug wich prevents to open the GUI directly. It is necessary to execute from the command window: >>ScaraRobot
- PanelRobot.m. MatLab script corresponding to ScaraRobot.fig.
Paths for inverse kinematics:
- Flamenco.txt (Flamingo)
- Gallo.txt (Rooster)
- mano.txt (Hand)
- MatLabLogo.txt (MatLab's logo)
- Pinguino.txt (Penguin)
- poligono.txt (Poligon)
- Lihour_1.txt (Lihour)

Table for forward kinematics (drive/r's angles)
- angulos.txt

Functions to process images and paths (.png, .mat, .txt)
- createDrawing.m. Main function
- circles.m. plots circles of radius r at points x and y. Chad Greene, March 2014. University of Texas Institute for Geophysics.
- fiveRmic.m. Maximal Inscribed Circle within the workspace of a 5R symetrical parallel mechanism.

SCARA.v1 Robot GUI by:
- Mr. SAN Lihour, Mecatronic Engineer
- AUTOBOTx Lab, AI FARM Factory
- Phnom Penh, Cambodia
- Platform:   GUI MATLAB R2020a
- January 2024

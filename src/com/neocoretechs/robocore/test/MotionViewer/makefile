OBJECTS = DrawInterface.class \
	FPoint.class \
	FloatCanvas.class \
	IdealDrive.class \
	JFBSlider.class \
	MotionApplet.class \
	Position.class \
	PositionPanel.class

%.class: %.java
	javac $<


.PHONY: all cleanup doc
all: $(OBJECTS)
	echo -e 'Done\n\n\n'

doc:
	javadoc -author -version -use *.java

cleanup:
	rm *.class *.html



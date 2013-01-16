package edu.wpi.first.smartdashboard.extension.tutorial;

public class TutorialLabel {
	
};

public class TutorialLabel extends StaticWidget {
	
	JLabel label;
	
	@Override
	public void init() {
		
		add(label);
	}
	@Overide
	public void propertyChanged(Property property) {
	}
	
}

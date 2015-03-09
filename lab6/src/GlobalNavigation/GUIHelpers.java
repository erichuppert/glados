package GlobalNavigation;
import java.awt.Color;
import java.util.Random;

import org.ros.message.lab5_msgs.*;

public class GUIHelpers {
	
	private Random rand = new Random();
	
	public static Color randomColor() {
		float r = rand.nextFloat();
		float g = rand.nextFloat();
		float b = rand.nextFloat();
		Color rndC = new Color(r,g,b);
		return rndC.darker();
	}
	
	public static ColorMsg colorMessage(Color color) {
		ColorMsg msg = new ColorMsg();
		msg.r = color.getRed();
		msg.b = color.getBlue();
		msg.g = color.getGreen();
		return msg;
	}
	
	public static ColorMsg randomColorMsg() {
		return colorMessage(randomColor()); 
	}
}

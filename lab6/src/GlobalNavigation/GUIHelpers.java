package GlobalNavigation;
import java.awt.Color;

import LocalNavigation.ColorMsg;

public class GUIHelpers {
	
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

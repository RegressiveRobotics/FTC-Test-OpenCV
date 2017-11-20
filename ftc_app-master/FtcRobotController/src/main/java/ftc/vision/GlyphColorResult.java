package ftc.vision;

import org.opencv.core.Scalar;

/**
 * Created by Bobby on 11/19/2017.
 */

public class GlyphColorResult {
    public GlyphColorResult(GlyphColor leftColor, GlyphColor rightColor) {
        this.leftColor = leftColor;
        this.rightColor = rightColor;
    }

    public enum GlyphColor {
        GRAY (ImageUtil.GRAY),
        BROWN (ImageUtil.BROWN),
        UNKNOWN (ImageUtil.BLACK);

        public final Scalar color;

        GlyphColor(Scalar color){
            this.color = color;
        }
    }
    private final GlyphColor leftColor, rightColor;
    @Override
    public String toString(){
        return leftColor + "," +rightColor;
    }
    public GlyphColor getLeftColor() {
        return leftColor;
    }

    public GlyphColor getRightColor() {
        return rightColor;
    }
}

package org.firstinspires.ftc.aperture.libraries;

import android.graphics.Bitmap;

import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

/**
 * VuforiaLocalizerImplSubclass
 *
 * Used to get access to a bitmap of what the camera is seeing.
 */

public class VuforiaLocalizerImplSubclass extends VuforiaLocalizerImpl {

    public Bitmap bitmap = null;
    public boolean copyNextToBitmap = false;

    class CloseableFrame extends Frame {
        public CloseableFrame(Frame other) {
            super(other);
        }
        public void close() {
            super.delete();
        }
    }

    public class VuforiaCallbackSubclass extends VuforiaCallback {

        @Override
        public synchronized void Vuforia_onUpdate(State state) {
            super.Vuforia_onUpdate(state);
            CloseableFrame frame = new CloseableFrame(state.getFrame());
            long num = frame.getNumImages();
            Image image = null;
            for (int i = 0; i < num; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    image = frame.getImage(i);
                }
            }

            if (copyNextToBitmap && image != null) {

                System.out.println("VUFORIA: Going to create a bitmap (" + image.getWidth()
                        + "," + image.getHeight() + ")");

                // create the bitmap once and only once
                if (bitmap == null) {
                    bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(),
                            Bitmap.Config.RGB_565);
                }

                // fill in the bitmap from the image
                bitmap.copyPixelsFromBuffer(image.getPixels());

                // wait until the next one is requested
                copyNextToBitmap = false;
            }

            frame.close();
        }
    }

    public VuforiaLocalizerImplSubclass(Parameters parameters) {
        super(parameters);
        stopAR();
        clearGLSurface();
        this.vuforiaCallback = new VuforiaCallbackSubclass();
        startAR();
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

    public void clearGLSurface() {
        if (this.glSurfaceParent != null) {
            appUtil.synchronousRunOnUiThread(new Runnable() {
                @Override
                public void run() {
                    glSurfaceParent.removeAllViews();
                    glSurfaceParent.getOverlay().clear();
                    glSurface = null;
                }
            });
        }
    }
}
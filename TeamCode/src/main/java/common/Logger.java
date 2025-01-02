/**
 * This class contains methods to log messages to Android Studio's Logcat window.
 */

package common;

import android.util.Log;

public final class Logger {
    public static final boolean VERBOSE = true;
    private static final String TAG = "DELMAR";

    public static String getCaller () {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        return String.format("%-16s", caller);
    }

    private static void logString (String str) {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        Log.d(TAG, String.format("%-24s %-24s %s", Thread.currentThread().getName(), caller, str));
    }

    public static void error (Exception e, String msg) {
        Log.e(TAG, "\n");
        Log.println(Log.ERROR, TAG, msg);
        Log.e(TAG, e.getMessage());

        StackTraceElement[] stackTraceElements = e.getStackTrace();
        for (int i=0; i<Math.min(stackTraceElements.length, 10); i++)
            Log.e(TAG, stackTraceElements[i].toString());
    }

    public static void warning(String warning) {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        Log.w(TAG, String.format("%-24s %s", caller, warning));
    }

    public static void warning(String format, Object... args) {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        Log.w(TAG, String.format("%-24s %s", caller, String.format(format, args)));
    }

    public static void message(String msg){
        logString(msg);
    }

    public static void message(String format, Object... args){
        logString(String.format(format, args));
    }

    public static void verbose(String format, Object... args) {
        if (VERBOSE) {
            logString(String.format(format, args));
        }
    }

    public static void addLine(String msg) {
        Log.d(TAG, msg);
    }
}

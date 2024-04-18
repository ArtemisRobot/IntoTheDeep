package common;

import android.util.Log;

/**
 *
*/

public final class Logger {
    private static final String TAG = "DELMAR";

    public static String getCaller () {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        return String.format("%-24s", caller);
    }

    private static void logString (String str) {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        Log.d(TAG, String.format("%-24s %s", caller, str));
    }

    public static void error (Exception e, String msg) {
        Log.println(Log.ERROR, TAG, msg);
        Log.e(TAG, e.getMessage());
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

    public static void message( String format, Object... args){
        logString(String.format(format, args));
    }

    public static void addLine(String msg) {
        Log.d(TAG, msg);
    }
}

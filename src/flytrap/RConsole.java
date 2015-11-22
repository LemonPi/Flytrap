package flytrap;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.*;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

public class RConsole extends Thread
{
    static final int MODE_SWITCH = 0xff;
    static final int MODE_LCD = 0x0;
    
    static final int RCONSOLE_PORT = 8001;    
    Socket conn = null;
    PrintStream origOut = System.out, origErr = System.err;
    public volatile boolean connected = false;
    private Deque<String> receiveDeque = new ArrayDeque<String>();
    SynchronizedOutputStream os;
    public PrintStream out = System.out;
    
    private TextLCD lcd = LocalEV3.get().getTextLCD();
    
	public RConsole()
	{
		super();
        setDaemon(true);
	}
	
	public boolean isConnected() {
		return (conn != null && conn.isConnected());
	}
	
	public String getLine() {
		synchronized(receiveDeque) {
			return receiveDeque.poll();
		}
	}
	public String waitLine() {
		String line;
		while (true) {
			line = getLine();
			if (line != null && line.trim().length() > 0) return line;
			Flytrap.sleep(10);
		}
	}

    /**
     * Main console I/O thread.
     */
    @Override
    public void run()
    {      
    	// Loop accepting remote console connections
    	while (true) {
            try {
            	System.out.println("Waiting for a connection");
        		conn = new Socket("10.0.1.10", 3334);
        		conn.setSoTimeout(2000);
        		os = new SynchronizedOutputStream(conn.getOutputStream());
        		BufferedReader input = new BufferedReader(new InputStreamReader(conn.getInputStream()));
        		//System.setOut(new PrintStream(os));
        		out = new PrintStream(os);
        		out.println("Client Hello");
        		//System.setErr(new PrintStream(os));
        		System.out.println("Output redirected");
        		connected = true;
                
                // Loop waiting for commands
                while (true)
                {
                    try {
                    	String line = input.readLine();
                    	if (line == null) break;
                    	synchronized(receiveDeque) {
                    		receiveDeque.add(line);
                    	}
                    } catch (SocketTimeoutException e) {
                    }
                }
                os.close();
                input.close();
                conn.close();
                //System.setOut(origOut);
                out = System.out;
                //System.setErr(origErr);
                System.out.println("System output set back to original");
                connected = false;
            }
            catch(IOException e)
            {
            	System.err.println("Error accepting connection " + e);
            }
    	}
    }
}

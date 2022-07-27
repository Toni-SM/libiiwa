package application;

import com.kuka.task.ITaskLogger;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;

import java.nio.ByteBuffer;

import application.LibIiwaEnum;


public class LibIiwaCommunication {

	public LibIiwaEnum propCommunicationErrorEnum;
	
	private int propInputLength = 0;
	private int propInputBytesLength = 0;
	private int propSocketInputTimeout = 0;
	private ITaskLogger propLogger = null;
	
	private Socket propCommunicationSocket = null;
	private ServerSocket propCommunicationServerSocket = null;
	private DataInputStream propCommunicationDataInputStream = null;
	private DataOutputStream propCommunicationDataOutputStream = null;
	
	// ===========================================================
	// PRIVATE METHODS
	// ===========================================================
	
	@SuppressWarnings("unused")
	private byte[] methDoubleToByteArray(double d) {
    	byte[] array = new byte[8];
    	ByteBuffer.wrap(array).putDouble(d);
    	return array;
    }
    
    @SuppressWarnings("unused")
	private double methByteArrayToDouble(byte[] bytes) {
    	return ByteBuffer.wrap(bytes).getDouble();
    }
    
    private byte[] methDoubleArrayToByteArray(double[] doubles) {
    	ByteBuffer buffer = ByteBuffer.allocate(doubles.length * 8);
    	for(double d : doubles)
    		buffer.putDouble(d);
    	return buffer.array();
    }
    
    private double[] methByteArrayToDoubleArray(byte[] bytes) {
    	ByteBuffer buffer = ByteBuffer.wrap(bytes);
    	double[] doubles = new double[bytes.length / 8];
    	for(int i = 0; i < doubles.length; i++)
    		doubles[i] = buffer.getDouble();
    	return doubles;
    }
    
	// ===========================================================
	// PUBLIC METHODS
	// ===========================================================
	
	public LibIiwaCommunication(ITaskLogger logger, int inputLength) {
		this.propLogger = logger;
		this.propInputLength = inputLength;
		this.propInputBytesLength = inputLength * 8;
	}
	
	public boolean methInitializeSocketConnection(String address, int port, boolean asServer, int blockingTimeout, int socketTimeout) {
		this.propSocketInputTimeout = socketTimeout;
		
		// server
		if(asServer) {
			try {
				this.propCommunicationServerSocket = new ServerSocket(port);
				this.propCommunicationServerSocket.setSoTimeout(blockingTimeout);
				this.propLogger.info("Server is listening on port " + port);
				this.propLogger.info("Waiting for client (" + blockingTimeout / 1000.0 + " seconds timeout)");
			} catch (IOException e) {
				this.propLogger.error("Server socket initialization error: " + e.toString());
	            return false;
	        }
			// wait for client connection
			try {
				this.propCommunicationSocket = this.propCommunicationServerSocket.accept();
				this.propCommunicationSocket.setSoTimeout(this.propSocketInputTimeout);
			} catch (Exception e) {
				if(e instanceof SocketTimeoutException)
					this.propLogger.warn("Client connection timeout reached: " + blockingTimeout / 1000.0 + " seconds");
				else if(e instanceof IOException)
					this.propLogger.error("Socket initialization error: " + e.toString());
				return false;
			}
		}
		// client
		else{
			try {
				this.propLogger.info("Connecting to " + address + ":" + port);
				this.propCommunicationSocket = new Socket(address, port);
			} catch (Exception e) {
				this.propLogger.error("Socket initialization error: " + e.toString());
				return false;
			}
		}
		// create data streams
		try {
			this.propCommunicationDataInputStream = new DataInputStream(this.propCommunicationSocket.getInputStream());
			this.propCommunicationDataOutputStream = new DataOutputStream(this.propCommunicationSocket.getOutputStream());
		
		} catch (IOException e) {
			this.propLogger.error("Socket data stream error: " + e.toString());
			return false;
		}
		return true;
    }
	
	public boolean methSetSocketTimeout(int socketTimeout){
		this.propSocketInputTimeout = socketTimeout;
		try {
			this.propCommunicationSocket.setSoTimeout(this.propSocketInputTimeout);
		} catch (Exception e) {
			this.propLogger.error("Socket timeout configuration: " + e.toString());
			return false;
		}
		return true;
	}
	
	public void methCloseSocketConnection() {
		if(this.propCommunicationSocket != null) {
			try {
				this.propCommunicationSocket.close();
			} catch (IOException error) {
			}
		}
		if(this.propCommunicationServerSocket != null) {
			try {
				this.propCommunicationServerSocket.close();
			} catch (IOException error) {
			}
		}
		this.propCommunicationSocket = null;
		this.propCommunicationServerSocket = null;
	}
	
	public boolean methSendData(double[] data) {
		try {
			this.propCommunicationDataOutputStream.write(this.methDoubleArrayToByteArray(data));
			this.propCommunicationDataOutputStream.flush();
		} catch (IOException e) {
			this.propLogger.error("Output stream error: " + e.toString());
			return false;
		}
		return true;
	}
	
	public double[] methReceiveData() {
		byte[] bytes = new byte[this.propInputBytesLength];
		double[] error = new double[this.propInputLength];
		try {
			int bytesRead = this.propCommunicationDataInputStream.read(bytes, 0, this.propInputBytesLength);
			// parse and return data
			if(bytesRead == this.propInputBytesLength)
				return this.methByteArrayToDoubleArray(bytes);
			// timeout (TIMEOUT)
			else if(bytesRead == -1) {
				error[0] = LibIiwaEnum.TIMEOUT.getCode();
				error[1] = this.propSocketInputTimeout;
			}
			// invalid number of bytes (INVALID_NUMBER_OF_BYTES)
			else {
				error[0] = LibIiwaEnum.INVALID_NUMBER_OF_BYTES.getCode();
				error[1] = bytesRead;
			}
			return error;
		} catch (IOException e) {
			this.propLogger.error("Input stream error: " + e.toString());
		}
		// return error with code EXCEPTION
		error[0] = LibIiwaEnum.EXCEPTION.getCode();
		return error;
	}
	
	public void methLogError(double[] error) {
		double code = error[0];
		if(code == LibIiwaEnum.EXCEPTION.getCode())
			this.propLogger.error("Communication error: Exception");
		else if(code == LibIiwaEnum.INVALID_NUMBER_OF_BYTES.getCode())
			this.propLogger.error("Communication error: Received/Expected bytes: " + (int)error[1] + "/" + this.propInputBytesLength);
		else if(code == LibIiwaEnum.TIMEOUT.getCode())
			this.propLogger.error("Communication error: Timeout reached: " + (int)error[1] + " milliseconds");
	}
}
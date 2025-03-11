import py4j.GatewayServer;

public class MyJavaApp {
    private int a;  // Declare 'a' as a private member variable

    // Constructor with an int argument
    public MyJavaApp() {
        this.a = 0;  // Default value for 'a'
    }

    // Setter method to allow Python code to set the value of 'a'
    public void setA(int a) {
        this.a = a;
    }

    // Method to get the value of 'a'
    public int getA() {
        return this.a;
    }

    // Method to be called from Python
    public void test() {
        System.out.println("Running Java code: " + getA());
    }

    public static void main(String[] args) {
        // Initialize MyJavaApp without arguments
        MyJavaApp app = new MyJavaApp();
        
        // Start the Py4J Gateway Server
        GatewayServer server = new GatewayServer(app, 25333); // Port 25333
        server.start();
        System.out.println("Py4J Gateway Server Started...");
    }
}

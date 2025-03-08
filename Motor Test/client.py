from py4j.java_gateway import JavaGateway

# Connect to the Java Gateway Server
gateway = JavaGateway()

# Access the Java class and create multiple objects
java_object1 = gateway.jvm.MyJavaApp()  # Create the first Java object
java_object2 = gateway.jvm.MyJavaApp()  # Create the second Java object
java_object3 = gateway.jvm.MyJavaApp()  # Create the third Java object

# Set different values for 'a' for each object
java_object1.setA(10)  # Set 'a' of object1 to 10
java_object2.setA(20)  # Set 'a' of object2 to 20
java_object3.setA(30)  # Set 'a' of object3 to 30

print("Testing objects: \n")
java_object1.test()
java_object2.test() 
java_object3.test() 

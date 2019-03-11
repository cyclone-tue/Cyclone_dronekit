from onehoop import flight

# Method for indexing array as if it is a matrix.
def matrix_index(a, rowsize, m, n):
    return(a[rowsize * m + n])


if __name__ == "__main__":
    myFlight = flight()
    try:
        while True:
            state = myFlight.drone.get_state()
            torques = myFlight.drone.get_torques_and_thrust()
            myFlight.logger.debug("state is {}", state)
            myFlight.logger.debug("thrust, torques are {}", state)
    except KeyboardInterrupt:
        myFlight.logger.info("Ending")
    myFlight.end()
    del myFlight

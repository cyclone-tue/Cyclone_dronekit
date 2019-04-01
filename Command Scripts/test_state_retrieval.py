from onehoop import flight
import time

# Method for indexing array as if it is a matrix.
def matrix_index(a, rowsize, m, n):
    return(a[rowsize * m + n])


if __name__ == "__main__":
    myFlight = flight()
    try:
        while True:
            state = myFlight.drone.get_state()
            torques = myFlight.drone.get_torques_and_thrust()
            #print(state[0])
            #myFlight.logger.debug("this {}".format(state[0]))
            state = [ '%.2f' % elem for elem in state]
            torques = [ '%.2f' % elem for elem in torques]
            myFlight.logger.debug("state is [{}, {}, {},  {}, {}, {},  {}, {}, {},  {}, {}, {}]".format(state[0], state[1], state[2], state[3] ,state[4] ,state[5] ,state[6] ,state[7], state[8], state[9], state[10], state[11]))
            myFlight.logger.debug("thrust, torques are [{}, {}, {}, {}]".format( torques[0], torques[1], torques[2], torques[3]))
            time.sleep(0.02)
    except KeyboardInterrupt:
        myFlight.logger.info("Ending")
    myFlight.end()
    del myFlight

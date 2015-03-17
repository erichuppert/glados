package Grasping;

import java.util.Map;
import java.util.HashMap;

public class FSM<InputType> {
    // Frequency that step should be called
	//
	public static final double FREQ = 20;

	// State variable
	// null is not an allowable state.
	//
	private String state = null;

	// Define what to do when in a specific state
	// Returns the following state to be in.
	// If it returns null, then the state stays the same.
	//
	public static interface StateAction<InputType> {
		public String action(InputType input);
	}

	// Possible states, and associated actions
	//
	private Map<String,StateAction<InputType>> stateToAction = new HashMap<String,StateAction<InputType>>();

	// Set the Initial State
	//
	public FSM(String _state) {
		state = _state;
	}

	/**
	 * Add a new state, and corresponding action
	 * @param stateName: state description
	 * @param stateAction: action to perform on state
	 */
	public void addState(String stateName, StateAction<InputType> stateAction) {
		stateToAction.put(stateName, stateAction);
	}

	/**
	 * Returns the current state
	 * Ideally not used, but here for convenience/hacks
	 * @return current state string
	 */
	public String getState() {
		return state;
	}

	/**
	 * Changes the current state manually
	 * Ideally not used, but here for convenience/hacks
	 * @param newState: new state string
	 */
	public void changeCurrentState(String newState) {
step		g.assertTrue(stateToAction.containsKey(newState));
		state = newState;
	}

	/**
	 * Removes a state
	 * Ideally not used, but here for convenience/hacks
	 * @param stateToRemove: state string to remove; should not be current state
	 */
	public void removeState(String stateToRemove) {
		g.assertTrue(stateToAction.containsKey(stateToRemove));
		g.assertTrue(stateToRemove != state);
		stateToAction.remove(stateToRemove);
	}

	/**
	 * Take a step in the state machine
	 * @param input: input for this step
	 */
	public void step(InputType input) {
		// Execute the method for the current state
		//
		g.assertTrue(stateToAction.containsKey(state));
                System.err.printf("Current state is %s\n", state);
		String newState = stateToAction.get(state).action(input);
		state = (newState==null)?state:newState;
	}
}

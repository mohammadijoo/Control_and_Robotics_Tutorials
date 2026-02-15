def update_trust(successes, lam=0.2, T0=0.5):
    """
    successes: list of 0/1 values (perceived failures/successes)
    lam: trust update gain (0 < lam <= 1)
    T0: initial trust level
    """
    T = T0
    history = [T]
    for s in successes:
        T = (1.0 - lam) * T + lam * float(s)
        history.append(T)
    return history

if __name__ == "__main__":
    # Example: 1=success, 0=failure over 10 interactions
    seq = [1, 1, 0, 1, 1, 1, 0, 1, 1, 1]
    trust_traj = update_trust(seq, lam=0.3, T0=0.2)
    for k, T in enumerate(trust_traj):
        print(f"Step {k}: T = {T:.3f}")
      

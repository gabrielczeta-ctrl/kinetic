KINETIC duel - 1v1 with a friend (experimental)
================================================

Both of you need this KINETIC folder (the same build) and your own Morrowind.

1. Connect your PCs
   Same house: nothing to do, use the LAN address (ipconfig).
   Over the internet: both install Tailscale (https://tailscale.com, free), sign in,
   and share your machine with your friend (or both join the same tailnet).
   The host's address is then "tailscale ip -4" (a 100.x.y.z number).
   Without Tailscale the host must forward UDP port 27015 on the router.

2. Start
   Host:  run KINETIC-Duel.cmd, choose 1, type your name.
   Guest: run KINETIC-Duel.cmd, choose 2, type the host's address and your name.
   Both start on a flat test floor, facing each other, with a steel longsword
   and the same stats. The bar at the top shows the connection, the score, the
   opponent's health and the ping.

3. Fight
   Draw (F), then fight as in KINETIC: hold the left button and move the mouse
   to steer the blade, click to swing, right button to guard.
   A round ends when someone drops to 1 health; after 4 seconds both are healed
   and put back. Nobody dies.

How it works: each game runs its own fighter at full speed; the opponent is a
copy of their body, replayed about 0.1 s behind. Your game decides your hits on
them and sends the damage and the push; theirs decides theirs on you.

Known limits of this first version: one weapon (steel longsword); the copy's
blade blocks and clashes with yours, but a parry only knocks back the body of
the player whose game saw it; no wound marks on the copy; the copy looks like
your own character, not your friend's.

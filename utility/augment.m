function z = augment(z1, z2)

c = [z1.center; z2.center];
G = [z1.generators; z2.generators];

z = zonotope(c,G);

end
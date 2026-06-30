module ts06(){
translate([-235,0,0])
translate([220,0,0])
hull(){
translate([0,0,40])cube([440,80,0.1],center=true);
translate([0,0,-40])cube([440,80,0.1],center=true);
translate([250,0,0])sphere(r=25);
}

module wing(){
rotate([0,0,160+90])
translate([540/4,0,-40])cube([540/2,80,4],center=true);
}



for (y=[205/2,205/2,-205/2,-205/2])
for (z=[205/2,-205/2,205/2,-205/2])
  
translate([-235+20,0,0])
union(){
hull(){
    rotate([0,90,0])translate([0,0,-10/2])cylinder(r=10,h=10,$fn=40);
    translate([0,y,z])rotate([0,90,0])translate([0,0,-10/2])cylinder(r=10,h=10,$fn=40);
}
translate([0,y,z])rotate([0,90,0])translate([0,0,-10/2])cylinder(r=7*25/2,h=1,$fn=40);
translate([-15,y,z])rotate([0,90,0])translate([0,0,-10/2])cylinder(r=30/2,h=30,$fn=40);
}
translate([0,0,-1])
union(){
mirror([0,0,0])wing();
mirror([0,1,0])wing();
}
}

scale(0.001)ts06();
use File::Basename;
#use warnings;
use Cwd;
open(FILE,"<",@ARGV[0]) or die "cannot open file";
@Lines=<FILE>;
close(FILE);

open(FILE,">",@ARGV[1]);
#$concatLines=join('',@Lines);
foreach(@Lines){
    if ($_ =~ m:dn1\s*?,\s*?(\d+)\/100\s*?,\s*?(\d+)\/100:g) { #Text after first match
	  $MatchText='dn1,'.$1.','.$2;
	} else{
	  $MatchText='dn1,000,000';
	}
	$MatchText=$MatchText."\n";
	print FILE "$MatchText";
	$MatchText = '';	
}
close(FILE);

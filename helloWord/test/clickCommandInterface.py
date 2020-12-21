import click

@click.command()
@click.option('-s', '--show', is_flag=True,                             help="Show image window")
@click.option('-w', '--width', default=3280, show_default=True,         help="Camera capture width (pixel)  ")
@click.option('-h', '--height', default=2464, show_default=True,        help="Camera capture height (pixel) ")
@click.option('-W', '--displaywidth', default=820, show_default=True,   help="Image display width (pixel)   ")
@click.option('-H', '--displayheight', default=616, show_default=True,  help="Image display height (pixel)  ")
@click.option('-r', '--framerate', default=20, show_default=True,       help="Image framerate")
@click.option('-f', '--flip', default=0, show_default=True,             help="Flip Method (see usage)")
def main(show, width, height, displaywidth, displayheight, framerate, flip):
  """
  \b
  Flip Method:
    (0): none             - Identity (no rotation)
    (1): counterclockwise - Rotate counter-clockwise 90 degrees
    (2): rotate-180       - Rotate 180 degrees
    (3): clockwise        - Rotate clockwise 90 degrees
    (4): horizontal-flip  - Flip horizontally
    (5): upper-right-diagonal - Flip across upper right/lower left diagonal
    (6): vertical-flip    - Flip vertically
    (7): upper-left-diagonal - Flip across upper left/low")
  """

  global showImage
  showImage = True if show else False
  if (show):
    print "Show"
  else: 
    print "Not show"






if __name__ == "__main__":
    main()